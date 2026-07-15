#!/usr/bin/env python3
"""DG Control Service (DCS) — 시나리오1 순찰 오케스트레이터 본체.

시퀀스 다이어그램(Confluence page 23691289, 2026-07-14 개정)의 DCS 역할을 구현한다.

  E0 상시 모니터링:
    - 구독  /{robot_id}/ddago/telemetry (DdagoTelemetry)
    - 구독  /{robot_id}/ddagi/telemetry (DdagiTelemetry)
    - 발행  /automato/telemetry/fleet    (FleetTelemetry, 1Hz 취합)
  E1/E2 순찰(경로 하달):
    - 액션 서버      /{robot_id}/navigate        (Navigate, Waypoint[]) ← Automato Control Service
    - 액션 클라이언트 /{robot_id}/ddago/navigate  (Navigate, Waypoint[]) → DdaGo Control Service
  E2 촬영·분석·저장:
    - 서비스 서버    /dg/analyze_frame           (AnalyzeFrame) ← DdaGo (capture 노드 도착 후)
    - TCP 클라이언트  DG AI Service               (4B len+JSON)  → 분석 위임
    - 서비스 클라이언트 /automato/save_detection  (SaveDetection)→ Automato Control Service

DCS 는 **중계자**다(다이어그램 E2-6: "DG는 중계만 한다").
  ACS 가 예약 확보된 구간을 Waypoint[] 로 하달 → DCS 가 그대로 DdaGo 에 넘김
  → DdaGo 의 feedback(current_waypoint_id, waypoint_index, x, y, yaw)·result(result_code,
    last_waypoint_id)를 **그대로 즉시 ACS 로 중계** → ACS 가 재계획 후 다음 구간 하달.
  통로 예약/해제·BFS·막힘 판정·복귀는 전부 ACS 몫이라 DCS 에는 없다.
  ACS 의 취소(cancel goal)도 DdaGo 로 중계한다(E2 22-1).

  [병렬] capture==true 노드에 도착한 DdaGo 가 AnalyzeFrame 호출 → DCS 가 AI(TCP) 자문
        → 결과(percent + 병해충 라벨 이미지)를 SaveDetection 으로 ACS 에 전달.
        라벨 이미지 **파일 저장은 ACS 몫**이라 DCS 는 저장하지 않고 전달만 한다.

AI 접속 대상은 dg_web/dg_ai_target.json 의 active("real"|"sim")를 따른다(대시보드에서 전환).

실행:
  source /opt/ros/jazzy/setup.bash
  source install/setup.bash
  ros2 run dg_control dcs_node
"""
import base64
import io
import json
import threading
import time

import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from automato_interfaces.action import Navigate
from automato_interfaces.msg import FleetTelemetry
from automato_interfaces.srv import AnalyzeFrame, SaveDetection
from sensor_msgs.msg import Image

from dg_control.ai_client import AiTcpClient

# 이 워크스페이스의 dg_web/dg_ai_target.json 기본 경로(대시보드가 실/시뮬 IP·active 저장).
DEFAULT_AI_TARGET_FILE = (
    '/home/ane/dev_ws/roscamp-rp108-navigate/equip/automato_ws/dg_web/dg_ai_target.json')


class DcsNode(Node):
    def __init__(self, **kwargs):
        super().__init__('dg_control_dcs', **kwargs)

        self.declare_parameter('robot_id', 'dg_01')
        self.declare_parameter('fleet_hz', 1.0)
        self.declare_parameter('ai_target_file', DEFAULT_AI_TARGET_FILE)
        self.declare_parameter('ai_default_endpoint', '127.0.0.1:9100')
        # 신선한 로봇 텔레메트리가 이 시간(초) 넘게 안 오면 FleetTelemetry 발행 중지
        self.declare_parameter('fleet_stale_sec', 3.0)
        # DdaGo 가 구간(Waypoint[]) 하나를 끝낼 때까지 기다리는 상한(초)
        self.declare_parameter('ddago_result_timeout_sec', 180.0)

        self.robot_id = self.get_parameter('robot_id').value
        fleet_hz = float(self.get_parameter('fleet_hz').value)
        self._fleet_stale = float(self.get_parameter('fleet_stale_sec').value)
        self._ddago_timeout = float(self.get_parameter('ddago_result_timeout_sec').value)

        # 콜백 그룹: 서비스/타이머는 동시 처리(Reentrant), 액션 클라이언트는 순차(Exclusive).
        self._cb_re = ReentrantCallbackGroup()
        self._cb_client = MutuallyExclusiveCallbackGroup()

        # ---- E0 텔레메트리 ----
        self._ddago_tel = {}   # robot_id -> DdagoTelemetry
        self._ddagi_tel = {}   # robot_id -> DdagiTelemetry
        self._ddago_rx = {}    # robot_id -> 마지막 수신 시각
        self._ddagi_rx = {}
        from automato_interfaces.msg import DdagiTelemetry, DdagoTelemetry
        self.create_subscription(
            DdagoTelemetry, '/%s/ddago/telemetry' % self.robot_id,
            self._on_ddago_tel, 10, callback_group=self._cb_re)
        self.create_subscription(
            DdagiTelemetry, '/%s/ddagi/telemetry' % self.robot_id,
            self._on_ddagi_tel, 10, callback_group=self._cb_re)
        self._fleet_pub = self.create_publisher(
            FleetTelemetry, '/automato/telemetry/fleet', 10)
        self.create_timer(1.0 / fleet_hz, self._publish_fleet,
                          callback_group=self._cb_re)

        # ---- E1/E2 Navigate 액션 서버 (ACS ← ) ----
        self._navigate_srv = ActionServer(
            self, Navigate, '/%s/navigate' % self.robot_id,
            execute_callback=self._navigate_execute,
            cancel_callback=lambda _gh: CancelResponse.ACCEPT,
            callback_group=self._cb_re)

        # ---- E1/E2 Navigate 액션 클라이언트 (→ DdaGo, 경로 배열 그대로 중계) ----
        self._ddago_client = ActionClient(
            self, Navigate, '/%s/ddago/navigate' % self.robot_id,
            callback_group=self._cb_client)

        # ---- E2 AnalyzeFrame 서비스 서버 (DdaGo ← ) ----
        self.create_service(
            AnalyzeFrame, '/dg/analyze_frame', self._on_analyze_frame,
            callback_group=self._cb_re)

        # ---- E2 SaveDetection 서비스 클라이언트 (→ ACS) ----
        self._save_client = self.create_client(
            SaveDetection, '/automato/save_detection', callback_group=self._cb_re)

        # ---- E2 AI Service TCP 클라이언트 ----
        self.ai = AiTcpClient(
            target_file=self.get_parameter('ai_target_file').value,
            default_endpoint=self.get_parameter('ai_default_endpoint').value,
            logger=self.get_logger())

        self._req_seq = 0
        # 로봇 1대에 동시에 떠 있는 Navigate goal 은 하나뿐이어야 한다. 여러 goal 을 한
        # 액션 클라이언트로 겹쳐 보내면 rclpy 가 goal UUID 를 재사용해 결과가 뒤섞인다.
        # (ACS 가 겹쳐 하달하는 비정상 상황에서도 순서를 지키도록 DCS 에서 직렬화한다.)
        self._ddago_lock = threading.Lock()

        self.get_logger().info(
            'DCS 준비: robot_id=%s | Navigate서버 /%s/navigate | AI target=%s'
            % (self.robot_id, self.robot_id, self.get_parameter('ai_target_file').value))

    # 실제 오간 메시지 내용을 한 줄 JSON(@@WIRE@@)으로 남긴다. 대시보드가 읽어 표시.
    #   direction: 'to_dcs'(상대→DCS) | 'from_dcs'(DCS→상대)
    def _wire(self, direction, iface, payload):
        try:
            print('@@WIRE@@ ' + json.dumps(
                {'ts': time.time(), 'dir': direction, 'iface': iface, 'payload': payload},
                ensure_ascii=False, default=float), flush=True)
        except (TypeError, ValueError):
            pass

    @staticmethod
    def _msg_to_dict(msg):
        """ROS 메시지를 있는 그대로 dict 로 (필드 누락·반올림 없이, 실제 발행값 그대로).

        예외는 하나 — 이미지 픽셀(uint8[] data)처럼 아주 긴 배열은 원소를 전부 찍으면
        로그가 수십 MB 가 되므로 '<uint8[N]>' 요약으로 바꾼다."""
        def conv(v):
            if hasattr(v, 'get_fields_and_field_types'):           # 중첩 ROS 메시지
                return {f: conv(getattr(v, f)) for f in v.get_fields_and_field_types()}
            if isinstance(v, (bytes, bytearray)):
                return '<uint8[%d]>' % len(v)
            if isinstance(v, str) or isinstance(v, bool):
                return v
            if hasattr(v, '__len__') and not isinstance(v, dict):  # 배열(list/array/ndarray)
                if len(v) > 64:                                    # 이미지 픽셀 등
                    return '<uint8[%d]>' % len(v)
                return [conv(x) for x in v]
            if isinstance(v, int):
                return v
            if isinstance(v, float):
                return v
            try:                                                   # numpy float32/int32 등
                return float(v) if 'float' in type(v).__name__ else int(v)
            except (TypeError, ValueError):
                return str(v)
        return conv(msg)

    # ============================ E0 텔레메트리 ============================
    def _wrong_robot(self, kind, msg):
        """토픽 이름의 robot_id 와 메시지 안의 robot_id 가 다르면 무시하고 경고.

        둘이 어긋나면(예: 로봇은 dg_01 로 발행하는데 DCS 는 dg_02 로 떠 있음) 아무 에러 없이
        엉뚱한 로봇 데이터를 취합하게 된다. ROBOT_ID 환경변수 오설정을 바로 드러내기 위한 방어."""
        if msg.robot_id and msg.robot_id != self.robot_id:
            self.get_logger().warn(
                '%s 의 robot_id 불일치: 수신=%s, DCS=%s → 무시 (ROBOT_ID 환경변수 확인)'
                % (kind, msg.robot_id, self.robot_id))
            return True
        return False

    def _on_ddago_tel(self, msg):
        if self._wrong_robot('DdagoTelemetry', msg):
            return
        self._ddago_tel[msg.robot_id] = msg
        self._ddago_rx[msg.robot_id] = time.time()
        self._wire('to_dcs', 'DdagoTelemetry', self._msg_to_dict(msg))

    def _on_ddagi_tel(self, msg):
        if self._wrong_robot('DdagiTelemetry', msg):
            return
        self._ddagi_tel[msg.robot_id] = msg
        self._ddagi_rx[msg.robot_id] = time.time()
        self._wire('to_dcs', 'DdagiTelemetry', self._msg_to_dict(msg))

    def _publish_fleet(self):
        # 신선한(최근 fleet_stale 초 이내 수신) 로봇 텔레메트리만 취합.
        now = time.time()
        ddagos = [d for rid, d in self._ddago_tel.items()
                  if now - self._ddago_rx.get(rid, 0) < self._fleet_stale]
        ddagis = [d for rid, d in self._ddagi_tel.items()
                  if now - self._ddagi_rx.get(rid, 0) < self._fleet_stale]
        if not ddagos and not ddagis:
            return   # 신선한 텔레메트리 없음(E0 중지 등) → DCS→ACS 발행 중지
        msg = FleetTelemetry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.ddagos = ddagos
        msg.ddagis = ddagis
        self._fleet_pub.publish(msg)
        self._wire('from_dcs', 'FleetTelemetry', self._msg_to_dict(msg))

    # ===== E1/E2 Navigate 중계 — ACS 가 하달한 경로(Waypoint[])를 DdaGo 에 그대로 =====
    def _navigate_execute(self, goal_handle):
        """ACS 가 하달한 구간(Waypoint[])을 DdaGo 로 중계하고, DdaGo 의 feedback·result 를
        **그대로 즉시** ACS 로 전달한다. 촬영·분석·저장(E2)은 capture==true 노드에서 DdaGo 가
        AnalyzeFrame 으로 별도(병렬) 요청하므로 이 result 반환을 막지 않는다."""
        req = goal_handle.request
        wps = list(req.waypoints)
        task_id = req.task_id
        self.get_logger().info(
            '순찰 경로 수신(ACS→DCS): task=%d waypoints=%d capture=%s'
            % (task_id, len(wps), [w.waypoint_id for w in wps if w.capture]))
        self._wire('to_dcs', 'Navigate', self._msg_to_dict(req))

        code, last_wp, msg = self._drive_ddago(task_id, wps, goal_handle)

        result = Navigate.Result()
        result.result_code = int(code)
        result.last_waypoint_id = int(last_wp)
        result.message = msg or ''
        if code == 0:
            goal_handle.succeed()
        elif code == 2 and goal_handle.is_cancel_requested:
            goal_handle.canceled()
        else:
            goal_handle.abort()
        # 구간 결과를 즉시 ACS 로 전달 → ACS 가 재계획·다음 구간 하달 (분석·저장은 병렬 진행)
        self.get_logger().info('구간 결과 전달(DCS→ACS): task=%d code=%d last_wp=%d'
                               % (task_id, code, last_wp))
        self._wire('from_dcs', 'Navigate/result', self._msg_to_dict(result))
        return result

    def _drive_ddago(self, task_id, waypoints, up_gh):
        """DdaGo 에 Navigate(경로 배열) 하달 → 피드백을 ACS로 중계 → (code, last_wp, msg) 반환.
        goal 하나가 끝날 때까지 다음 goal 을 보내지 않는다(_ddago_lock)."""
        with self._ddago_lock:
            return self._drive_ddago_locked(task_id, waypoints, up_gh)

    def _drive_ddago_locked(self, task_id, waypoints, up_gh):
        last_wp = waypoints[0].waypoint_id if waypoints else -1
        if not self._ddago_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('DdaGo 액션 서버 없음')
            return 1, last_wp, 'DdaGo 서버 없음'

        goal = Navigate.Goal(task_id=task_id, waypoints=waypoints)
        self.get_logger().info('DdaGo 하달(DCS→DdaGo): task=%d waypoints=%d'
                               % (task_id, len(waypoints)))
        self._wire('from_dcs', 'Navigate(→DdaGo)', self._msg_to_dict(goal))

        def on_fb(fb_msg):
            fb = fb_msg.feedback
            nf = Navigate.Feedback()
            nf.current_waypoint_id = fb.current_waypoint_id
            nf.waypoint_index = fb.waypoint_index
            nf.current_x, nf.current_y, nf.current_yaw = fb.current_x, fb.current_y, fb.current_yaw
            self._wire('to_dcs', 'Navigate(→DdaGo)/feedback', self._msg_to_dict(fb))
            try:
                up_gh.publish_feedback(nf)   # DdaGo 피드백(도착 보고) → ACS 로 중계
                self._wire('from_dcs', 'Navigate/feedback', self._msg_to_dict(nf))
            except Exception:   # noqa: BLE001
                pass

        # goal 하달 → 수락 대기
        holder = {}
        acc_ev = threading.Event()
        sfut = self._ddago_client.send_goal_async(goal, feedback_callback=on_fb)
        sfut.add_done_callback(lambda f: (holder.__setitem__('gh', f.result()), acc_ev.set()))
        acc_ev.wait(timeout=5.0)
        gh = holder.get('gh')
        if gh is None or not gh.accepted:
            self.get_logger().error('DdaGo goal 거부/무응답')
            return 1, last_wp, 'DdaGo goal 거부'

        # 결과 대기. 그 사이 ACS 가 취소하면(E2 22-1) DdaGo goal 도 취소 중계.
        rholder = {}
        res_ev = threading.Event()
        gh.get_result_async().add_done_callback(
            lambda f: (rholder.__setitem__('r', f.result().result), res_ev.set()))
        waited = 0.0
        while not res_ev.wait(0.5):
            if up_gh.is_cancel_requested:
                self.get_logger().warn('ACS 취소 요청 → DdaGo goal 취소 중계')
                gh.cancel_goal_async()
            waited += 0.5
            if waited >= self._ddago_timeout:
                self.get_logger().warn('DdaGo 결과 무응답(timeout)')
                return 1, last_wp, 'DdaGo 결과 timeout'

        r = rholder.get('r')
        if r is None:
            return 1, last_wp, 'DdaGo 결과 없음'
        self.get_logger().info('DdaGo 구간 종료: result_code=%d last_wp=%d'
                               % (r.result_code, r.last_waypoint_id))
        self._wire('to_dcs', 'Navigate(→DdaGo)/result', self._msg_to_dict(r))
        return r.result_code, r.last_waypoint_id, r.message

    @staticmethod
    def _image_to_jpeg_b64(img):
        """sensor_msgs/Image(rgb8/bgr8/mono8) → JPEG base64. 실패 시 raw base64 폴백."""
        try:
            from PIL import Image as PILImage
            raw = bytes(img.data)
            if img.encoding == 'rgb8':
                pil = PILImage.frombytes('RGB', (img.width, img.height), raw)
            elif img.encoding == 'bgr8':
                r, g, b = PILImage.frombytes('RGB', (img.width, img.height), raw).split()[::-1]
                pil = PILImage.merge('RGB', (r, g, b))
            elif img.encoding in ('mono8', '8UC1'):
                pil = PILImage.frombytes('L', (img.width, img.height), raw)
            else:
                return base64.b64encode(raw).decode('ascii')   # 미지원 인코딩 → raw
            buf = io.BytesIO()
            pil.save(buf, format='JPEG', quality=85)
            return base64.b64encode(buf.getvalue()).decode('ascii')
        except Exception:   # noqa: BLE001
            try:
                return base64.b64encode(bytes(img.data)).decode('ascii')
            except (TypeError, ValueError):
                return ''

    # ============================ E2 분석·저장 ============================
    def _on_analyze_frame(self, request, response):
        """capture 노드 도착 후 DdaGo 의 분석 요청 접수. 즉시 accepted 응답하고 뒷처리는 백그라운드."""
        self._req_seq += 1
        request_id = 'req_%d_wp%d_%03d' % (request.task_id, request.waypoint_id, self._req_seq)
        response.accepted = True
        response.request_id = request_id

        # sensor_msgs/Image(raw) → JPEG base64 (스펙 image_encoding:"jpeg" 에 맞춤)
        image_b64 = self._image_to_jpeg_b64(request.image)
        wire = self._msg_to_dict(request)     # 이미지 픽셀(data)은 <uint8[N]> 로 요약됨
        wire['request_id'] = request_id          # DCS 가 부여(AI TCP 요청과 짝)
        wire['image_jpeg_b64_len'] = len(image_b64)
        self._wire('to_dcs', 'AnalyzeFrame', wire)

        threading.Thread(
            target=self._process_waypoint,
            args=(request.task_id, request.waypoint_id, request_id, image_b64),
            daemon=True).start()
        return response

    def _process_waypoint(self, task_id, waypoint_id, request_id, image_b64):
        # 3~4) DCS → AI(TCP) → 결과 (익음/덜익음/부패/병해 percent)
        #      ※ AI Service 는 rotten 과 disease 를 구분하지 못해 합쳐서 rotten 으로 보낸다.
        #        DCS 는 판정하지 않고 받은 4개 percent 를 그대로 ACS 로 전달한다.
        pct = {'ripe_percent': 0, 'unripe_percent': 0, 'rotten_percent': 0, 'disease_percent': 0}
        labeled = None   # AI 결과 라벨링 이미지(base64) — 있으면 SaveDetection.disease_image 로
        self._wire('from_dcs', 'analyze_request', {
            'message_type': 'analyze_frame_request', 'request_id': request_id,
            'task_id': task_id, 'waypoint_id': waypoint_id, 'image_encoding': 'jpeg',
            'image_data': '<base64 %d bytes>' % len(image_b64)})
        try:
            resp = self.ai.analyze(request_id, task_id, waypoint_id, image_data=image_b64)
            result = resp.get('result', {}) if isinstance(resp, dict) else {}
            pct.update({k: int(result.get(k, 0)) for k in pct})
            self.get_logger().info('분석결과 wp=%d ripe=%d unripe=%d rotten=%d disease=%d'
                                   % (waypoint_id, pct['ripe_percent'], pct['unripe_percent'],
                                      pct['rotten_percent'], pct['disease_percent']))
            # 라벨링 이미지: AI 가 disease_percent >= 5 일 때만 실어 보낸다(E3).
            # 응답 최상위/result 어느 쪽에 오든 받아서 ACS 로 그대로 넘긴다(저장은 ACS 몫).
            labeled = resp.get('labeled_image') or result.get('labeled_image')
            lenc = (resp.get('labeled_image_encoding') or result.get('labeled_image_encoding')
                    or 'jpeg')
            self._wire('to_dcs', 'analyze_response', {
                'message_type': 'analyze_frame_response', 'request_id': request_id,
                'status': 'OK', 'result': dict(pct),
                'labeled_image': ('<%s %d b64chars>' % (lenc, len(labeled))) if labeled else None})
        except Exception as e:   # noqa: BLE001 — 분석 실패해도 순찰은 계속(0 저장)
            self.get_logger().error('AI 분석 실패 wp=%d: %s' % (waypoint_id, e))
            self._wire('to_dcs', 'analyze_response', {
                'message_type': 'analyze_frame_response', 'request_id': request_id,
                'status': 'ERROR', 'error': str(e)})

        # 5) DCS → ACS SaveDetection (응답 대기 안 함). 라벨 이미지 있으면 함께.
        self._call_save_detection(task_id, waypoint_id, pct, labeled)

    @staticmethod
    def _jpeg_b64_to_image(b64):
        """base64 JPEG → sensor_msgs/Image(rgb8). 실패 시 None."""
        try:
            from PIL import Image as PILImage
            im = PILImage.open(io.BytesIO(base64.b64decode(b64))).convert('RGB')
            w, h = im.size
            img = Image()
            img.header.frame_id = 'labeled'
            img.height, img.width = h, w
            img.encoding = 'rgb8'
            img.is_bigendian = 0
            img.step = w * 3
            img.data = list(im.tobytes())
            return img
        except Exception:   # noqa: BLE001
            return None

    def _call_save_detection(self, task_id, waypoint_id, pct, labeled_b64=None):
        if not self._save_client.service_is_ready():
            self._save_client.wait_for_service(timeout_sec=2.0)
        req = SaveDetection.Request()
        req.task_id = int(task_id)
        req.waypoint_id = int(waypoint_id)
        req.robot_id = self.robot_id
        req.ripe_percent = pct['ripe_percent']
        req.unripe_percent = pct['unripe_percent']
        req.rotten_percent = pct['rotten_percent']
        req.disease_percent = pct['disease_percent']
        # AI 가 라벨 이미지를 보냈으면(disease_percent>=5) JPEG→Image 로 풀어 그대로 전달.
        # 없으면 빈 Image(height=0) → ACS 는 image_path 없이 저장.
        img = self._jpeg_b64_to_image(labeled_b64) if labeled_b64 else None
        img_wh = None
        if img is not None:
            req.disease_image = img
            img_wh = '%dx%d' % (img.width, img.height)
        wire = self._msg_to_dict(req)        # disease_image.data 는 <uint8[N]> 로 요약됨
        wire['disease_image_size'] = img_wh   # 없으면 None(=disease_percent<5)
        self._wire('from_dcs', 'SaveDetection', wire)
        self._save_client.call_async(req)   # fire-and-forget

    def destroy_node(self):
        try:
            self.ai.close()
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DcsNode()
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
