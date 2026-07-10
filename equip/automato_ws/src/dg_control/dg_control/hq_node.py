#!/usr/bin/env python3
"""DG Control Service (HQ) — 시나리오1 순찰 오케스트레이터 본체.

시퀀스 다이어그램(Confluence page 23691289)의 HQ 역할을 구현한다.

  E0 상시 모니터링:
    - 구독  /{robot_id}/ddago/telemetry (DdagoTelemetry)
    - 구독  /{robot_id}/ddagi/telemetry (DdagiTelemetry)
    - 발행  /automato/telemetry/fleet    (FleetTelemetry, 1Hz 취합)
  E1 순찰 시작:
    - 액션 서버      /{robot_id}/patrol          (Patrol, 단일 waypoint) ← Automato Control Service
    - 액션 클라이언트 /{robot_id}/ddago/patrol    (Patrol, 단일 waypoint) → DdaGo Control Service
  E2 웨이포인트 체크·저장 루프:
    - 서비스 서버    /dg/analyze_frame           (AnalyzeFrame) ← DdaGo (도착 후 분석요청)
    - TCP 클라이언트  DG AI Service               (4B len+JSON)  → 분석 위임
    - 서비스 클라이언트 /automato/save_detection  (SaveDetection)→ Automato Control Service

E2 한 waypoint 사이클 (루프 주체 = ACS, 도착 보고와 분석·저장은 병렬):
  ACS가 waypoint 1개를 Patrol로 하달 → HQ가 DdaGo에 Patrol(단일) 중계 → DdaGo 도착 →
  HQ가 DdaGo 피드백·도착 결과(Patrol Result)를 **즉시 ACS로 전달** → ACS가 다음 waypoint 하달.
  [병렬] DdaGo가 AnalyzeFrame 호출 → HQ가 AI(TCP) 자문 → 결과를 SaveDetection으로 ACS에 전달.

AI 접속 대상은 dg_web/dg_ai_target.json 의 active("real"|"sim")를 따른다(대시보드에서 전환).

실행:
  source /opt/ros/jazzy/setup.bash
  source install/setup.bash
  ros2 run dg_control hq_node
"""
import base64
import io
import json
import os
import threading
import time

import rclpy
from rclpy.action import ActionClient, ActionServer
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from automato_interfaces.action import Patrol
from automato_interfaces.msg import FleetTelemetry
from automato_interfaces.srv import AnalyzeFrame, SaveDetection
from sensor_msgs.msg import Image

from dg_control.ai_client import AiTcpClient

# 이 워크스페이스의 dg_web/dg_ai_target.json 기본 경로(대시보드가 실/시뮬 IP·active 저장).
DEFAULT_AI_TARGET_FILE = (
    '/home/ane/dev_ws/roscamp-sprint4-heeseog/equip/automato_ws/dg_web/dg_ai_target.json')
# AI 응답의 labeled_image(라벨링된 결과 이미지) 저장 폴더
DEFAULT_LABELED_DIR = (
    '/home/ane/dev_ws/roscamp-sprint4-heeseog/equip/automato_ws/labeled_recv')


class HqNode(Node):
    def __init__(self, **kwargs):
        super().__init__('dg_control_hq', **kwargs)

        self.declare_parameter('robot_id', 'dg_01')
        self.declare_parameter('fleet_hz', 1.0)
        self.declare_parameter('ai_target_file', DEFAULT_AI_TARGET_FILE)
        self.declare_parameter('ai_default_endpoint', '127.0.0.1:9100')
        # 신선한 로봇 텔레메트리가 이 시간(초) 넘게 안 오면 FleetTelemetry 발행 중지
        self.declare_parameter('fleet_stale_sec', 3.0)

        self.robot_id = self.get_parameter('robot_id').value
        fleet_hz = float(self.get_parameter('fleet_hz').value)
        self._fleet_stale = float(self.get_parameter('fleet_stale_sec').value)

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

        # ---- E1 Patrol 액션 서버 (ACS ← ) ----
        self._patrol_srv = ActionServer(
            self, Patrol, '/%s/patrol' % self.robot_id,
            execute_callback=self._patrol_execute,
            callback_group=self._cb_re)

        # ---- E1/E2 Patrol 액션 클라이언트 (→ DdaGo, 단일 waypoint) ----
        self._ddago_client = ActionClient(
            self, Patrol, '/%s/ddago/patrol' % self.robot_id,
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

        # ---- 순찰 상태 ----
        # ACS가 waypoint를 하나씩 하달 → HQ는 DdaGo 중계 후 도착 결과를 즉시 ACS로 전달.
        # E2 분석·저장은 AnalyzeFrame으로 별도(병렬) 처리 — result를 막지 않는다.
        self._req_seq = 0

        self.get_logger().info(
            'HQ 준비: robot_id=%s | Patrol서버 /%s/patrol | AI target=%s'
            % (self.robot_id, self.robot_id, self.get_parameter('ai_target_file').value))

    # 실제 오간 메시지 내용을 한 줄 JSON(@@WIRE@@)으로 남긴다. 대시보드가 읽어 표시.
    #   direction: 'to_hq'(서비스→HQ) | 'from_hq'(HQ→서비스)
    def _wire(self, direction, iface, payload):
        try:
            print('@@WIRE@@ ' + json.dumps(
                {'ts': time.time(), 'dir': direction, 'iface': iface, 'payload': payload},
                ensure_ascii=False, default=float), flush=True)
        except (TypeError, ValueError):
            pass

    @staticmethod
    def _hdr(header):
        """std_msgs/Header 를 표시용 dict 로."""
        return {'stamp': {'sec': header.stamp.sec, 'nanosec': header.stamp.nanosec},
                'frame_id': header.frame_id}

    # ============================ E0 텔레메트리 ============================
    def _on_ddago_tel(self, msg):
        self._ddago_tel[msg.robot_id] = msg
        self._ddago_rx[msg.robot_id] = time.time()
        self._wire('to_hq', 'DdagoTelemetry', {
            'header': self._hdr(msg.header),
            'robot_id': msg.robot_id, 'task_id': msg.task_id, 'nav_status': msg.nav_status,
            'is_charging': msg.is_charging, 'x': round(msg.x, 2), 'y': round(msg.y, 2),
            'yaw': round(msg.yaw, 2), 'battery_percent': round(msg.battery_percent, 1),
            'battery_voltage': round(msg.battery_voltage, 1), 'us_range_m': round(msg.us_range_m, 2)})

    def _on_ddagi_tel(self, msg):
        self._ddagi_tel[msg.robot_id] = msg
        self._ddagi_rx[msg.robot_id] = time.time()
        self._wire('to_hq', 'DdagiTelemetry', {
            'header': self._hdr(msg.header),
            'robot_id': msg.robot_id, 'task_id': msg.task_id, 'is_paused': msg.is_paused,
            'joint_angles': [round(float(v), 1) for v in msg.joint_angles],
            'tcp_coords': [round(float(v), 1) for v in msg.tcp_coords],
            'servo_health_count': len(msg.servo_health)})

    def _publish_fleet(self):
        # 신선한(최근 fleet_stale 초 이내 수신) 로봇 텔레메트리만 취합.
        now = time.time()
        ddagos = [d for rid, d in self._ddago_tel.items()
                  if now - self._ddago_rx.get(rid, 0) < self._fleet_stale]
        ddagis = [d for rid, d in self._ddagi_tel.items()
                  if now - self._ddagi_rx.get(rid, 0) < self._fleet_stale]
        if not ddagos and not ddagis:
            return   # 신선한 텔레메트리 없음(E0 중지 등) → HQ→ACS 발행 중지
        msg = FleetTelemetry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.ddagos = ddagos
        msg.ddagis = ddagis
        self._fleet_pub.publish(msg)
        self._wire('from_hq', 'FleetTelemetry', {
            'header': self._hdr(msg.header),
            'ddagos': [{'robot_id': d.robot_id, 'x': round(d.x, 2), 'y': round(d.y, 2),
                        'nav_status': d.nav_status, 'battery_percent': round(d.battery_percent, 1)}
                       for d in msg.ddagos],
            'ddagis': [{'robot_id': d.robot_id, 'is_paused': d.is_paused} for d in msg.ddagis]})

    # ========= E1 Patrol 실행 (단일 waypoint) — DdaGo 피드백·결과를 즉시 ACS로 중계 =========
    def _patrol_execute(self, goal_handle):
        """ACS가 하달한 단일 waypoint를 DdaGo로 중계하고, DdaGo의 피드백·도착 결과를 **그대로
        즉시** ACS로 전달한다. 분석·저장(E2)은 DdaGo의 AnalyzeFrame으로 별도(병렬) 진행되며
        Patrol result 반환을 막지 않는다. (도착 보고 → ACS가 다음 waypoint 하달)"""
        req = goal_handle.request
        wp = req.waypoint
        task_id = req.task_id
        self.get_logger().info('순찰 waypoint 수신: task=%d wp=%d (%.2f,%.2f)'
                               % (task_id, wp.waypoint_id, wp.x, wp.y))
        self._wire('to_hq', 'Patrol', {
            'task_id': task_id,
            'waypoint': {'waypoint_id': wp.waypoint_id,
                         'x': round(wp.x, 2), 'y': round(wp.y, 2)}})

        # DdaGo 주행 → 피드백은 _drive_ddago 안에서 즉시 ACS로 중계, 도착 결과를 받는다
        code, msg = self._drive_ddago(task_id, wp, goal_handle)
        result = Patrol.Result()
        result.result_code = code
        if code == 0:
            result.message = msg or '도착'
            goal_handle.succeed()
        else:
            result.message = msg or '주행 실패'
            goal_handle.abort()
        # 도착 결과를 즉시 ACS로 전달 (분석·저장은 병렬 진행)
        self.get_logger().info('waypoint 결과 전달(→ACS): task=%d wp=%d code=%d'
                               % (task_id, wp.waypoint_id, code))
        self._wire('from_hq', 'Patrol/result',
                   {'result_code': code, 'message': result.message})
        return result

    def _drive_ddago(self, task_id, waypoint, patrol_goal_handle):
        """DdaGo 에 Patrol(단일 waypoint) 하달 → 피드백을 ACS로 중계 → 도착 결과(code, msg) 반환."""
        if not self._ddago_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('DdaGo 액션 서버 없음')
            return 1, 'DdaGo 서버 없음'
        goal = Patrol.Goal(task_id=task_id, waypoint=waypoint)
        self.get_logger().info('DdaGo 하달: task=%d wp=%d (%.2f,%.2f)'
                               % (task_id, waypoint.waypoint_id, waypoint.x, waypoint.y))
        self._wire('from_hq', 'Patrol(→DdaGo)', {
            'task_id': task_id,
            'waypoint': {'waypoint_id': waypoint.waypoint_id,
                         'x': round(waypoint.x, 2), 'y': round(waypoint.y, 2)}})

        def on_fb(fb_msg):
            fb = fb_msg.feedback
            pf = Patrol.Feedback()
            pf.current_waypoint_id = fb.current_waypoint_id
            pf.current_x, pf.current_y, pf.current_yaw = fb.current_x, fb.current_y, fb.current_yaw
            self._wire('to_hq', 'Patrol(→DdaGo)/feedback ', {
                'current_waypoint_id': fb.current_waypoint_id,
                'current_x': round(fb.current_x, 2), 'current_y': round(fb.current_y, 2),
                'current_yaw': round(fb.current_yaw, 2)})
            try:
                patrol_goal_handle.publish_feedback(pf)   # DdaGo 피드백 → ACS 로 중계                
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
            return 1, 'DdaGo goal 거부'

        # 도착 결과 대기
        rholder = {}
        res_ev = threading.Event()
        gh.get_result_async().add_done_callback(
            lambda f: (rholder.__setitem__('r', f.result().result), res_ev.set()))
        res_ev.wait(timeout=25.0)
        r = rholder.get('r')
        if r is None:
            self.get_logger().warn('DdaGo 도착 무응답(timeout)')
            return 1, 'DdaGo 도착 timeout'
        self.get_logger().info('DdaGo 도착: result_code=%d' % r.result_code)
        self._wire('to_hq', 'Patrol(→DdaGo)/result',
                   {'result_code': r.result_code, 'message': r.message})
        return r.result_code, r.message

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

    # ============================ E2 분석·저장 루프 ============================
    def _on_analyze_frame(self, request, response):
        """DdaGo 도착 후 분석 요청 접수. 즉시 accepted 응답하고 뒷처리는 백그라운드로."""
        self._req_seq += 1
        request_id = 'req_%d_wp%d_%03d' % (request.task_id, request.waypoint_id, self._req_seq)
        response.accepted = True
        response.request_id = request_id

        # sensor_msgs/Image(raw) → JPEG base64 (스펙 image_encoding:"jpeg" 에 맞춤)
        image_b64 = self._image_to_jpeg_b64(request.image)
        self._wire('to_hq', 'AnalyzeFrame', {
            'task_id': request.task_id, 'waypoint_id': request.waypoint_id,
            'request_id': request_id, 'image_src': request.image.header.frame_id,
            'image_size': '%dx%d' % (request.image.width, request.image.height),
            'image_b64_len': len(image_b64)})

        threading.Thread(
            target=self._process_waypoint,
            args=(request.task_id, request.waypoint_id, request_id, image_b64),
            daemon=True).start()
        return response

    def _process_waypoint(self, task_id, waypoint_id, request_id, image_b64):
        # 3~4) HQ → AI(TCP) → 결과 (익음/덜익음/부패/병해 percent)
        pct = {'ripe_percent': 0, 'unripe_percent': 0, 'rotten_percent': 0, 'disease_percent': 0}
        labeled = None   # AI 결과 라벨링 이미지(base64), 있으면 SaveDetection에 실어보냄
        self._wire('from_hq', 'analyze_request', {
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
            # 라벨링 이미지(labeled_image): 응답 최상위 또는 result 안 어디든 대응, 있으면 저장
            labeled = resp.get('labeled_image') or result.get('labeled_image')
            lenc = (resp.get('labeled_image_encoding') or result.get('labeled_image_encoding')
                    or 'jpeg')
            saved = self._save_labeled_image(task_id, waypoint_id, labeled, lenc) if labeled else None
            self._wire('to_hq', 'analyze_response', {
                'message_type': 'analyze_frame_response', 'request_id': request_id,
                'status': 'OK', 'result': dict(pct),
                'labeled_image': ('<%s %d b64chars>' % (lenc, len(labeled))) if labeled else None,
                'labeled_saved': os.path.basename(saved) if saved else None})
        except Exception as e:   # noqa: BLE001 — 분석 실패해도 순찰은 계속(0 저장)
            self.get_logger().error('AI 분석 실패 wp=%d: %s' % (waypoint_id, e))
            self._wire('to_hq', 'analyze_response', {
                'message_type': 'analyze_frame_response', 'request_id': request_id,
                'status': 'ERROR', 'error': str(e)})

        # 5) HQ → ACS SaveDetection (응답 대기 안 함). 라벨 이미지 있으면 함께.
        #    E2 분석·저장은 도착 보고(Patrol result)와 병렬 — result는 이미 즉시 ACS로 전달됨.
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

    def _save_labeled_image(self, task_id, waypoint_id, b64, encoding):
        """AI 응답의 labeled_image(base64)를 파일로 저장. 저장 경로 반환(실패 시 None)."""
        try:
            raw = base64.b64decode(b64)
            os.makedirs(DEFAULT_LABELED_DIR, exist_ok=True)
            ext = 'jpg' if encoding == 'jpeg' else (encoding or 'bin')
            path = os.path.join(DEFAULT_LABELED_DIR,
                                'labeled_task%d_wp%d.%s' % (int(task_id), int(waypoint_id), ext))
            with open(path, 'wb') as f:
                f.write(raw)
            self.get_logger().info('AI 라벨 이미지 저장: %s (%d bytes)' % (path, len(raw)))
            return path
        except Exception as e:   # noqa: BLE001
            self.get_logger().warn('라벨 이미지 저장 실패: %s' % e)
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
        # AI 결과에 라벨 이미지가 있으면 JPEG→Image 로 채워 함께 저장 요청
        img = self._jpeg_b64_to_image(labeled_b64) if labeled_b64 else None
        img_wh = None
        if img is not None:
            req.disease_image = img
            img_wh = '%dx%d' % (img.width, img.height)
        self._wire('from_hq', 'SaveDetection', {
            'task_id': int(task_id), 'waypoint_id': int(waypoint_id),
            'robot_id': self.robot_id, **pct, 'image': img_wh})
        self._save_client.call_async(req)   # fire-and-forget

    def destroy_node(self):
        try:
            self.ai.close()
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = HqNode()
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
