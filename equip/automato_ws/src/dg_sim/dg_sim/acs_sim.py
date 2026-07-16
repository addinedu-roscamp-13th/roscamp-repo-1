#!/usr/bin/env python3
"""Automato Control Service 시뮬레이터.

팀원이 개발 중인 실제 Automato Control Service(ACS) 대역. 즉시-응답 스텁.

담당(시퀀스 다이어그램, 2026-07-14 개정):
  E0  FleetTelemetry 구독 (DCS ← )               /automato/telemetry/fleet   (로그로 확인)
  E1  Navigate 액션 클라이언트 (→ DCS)           /{robot_id}/navigate
        - **예약 확보된 구간까지만** Waypoint[] 로 하달(루프 주체=ACS)
        - 구간 result(last_waypoint_id) 받으면 다음 구간 하달 (E2 4단계)
        - 순찰 지점(capture=true)과 통과 노드(capture=false)를 섞어서 하달
  E2  SaveDetection 서비스 서버 (DCS ← )         /automato/save_detection    (즉시 success)
        - disease_image 가 있으면(=AI 가 disease>=5 로 판단) 파일로 저장 (저장은 ACS 몫)

실제 ACS 의 통로 예약(try_reserve)·BFS·막힘 판정은 흉내내지 않는다. 여기서는 경로를
seg_size 개씩 끊어 "예약된 구간까지만 하달"하는 형태만 재현한다.

편의:
  - 파라미터 auto_start(기본 true)면 기동 후 auto_delay 초에 순찰 1회 자동 발행.
  - 서비스 /acs_sim/start_patrol (std_srvs/Trigger) 호출로 언제든 순찰 재발행.
"""
import os

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from automato_interfaces.action import Navigate
from automato_interfaces.msg import FleetTelemetry, Waypoint
from automato_interfaces.srv import SaveDetection
from std_srvs.srv import Trigger

# SaveDetection 으로 받은 병해충 라벨 이미지 저장 폴더 (파일 저장은 ACS 담당)
ACS_SAVE_DIR = os.environ.get(
    'ACS_SIM_SAVE_DIR',
    '/home/ane/dev_ws/roscamp-rp108-navigate/equip/automato_ws/acs_recv')


class AcsSim(Node):
    def __init__(self, **kwargs):
        super().__init__('acs_sim', **kwargs)
        self.declare_parameter('robot_id', 'dg_01')
        self.declare_parameter('auto_start', True)
        self.declare_parameter('auto_delay', 3.0)
        self.declare_parameter('num_waypoints', 6)   # 경로 전체 노드 수
        self.declare_parameter('seg_size', 3)        # 한 번에 하달할 구간 크기(예약 흉내)
        self.robot_id = self.get_parameter('robot_id').value
        self._cb = ReentrantCallbackGroup()
        self._task_seq = 1024
        self.saved = []   # 수신한 SaveDetection 누적(검증/디버깅용)
        self.fleet_count = 0
        self.last_fleet = None
        self.last_result = None
        # 순찰 루프 상태: 구간(Waypoint[])을 하달하고 result를 받으면 다음 구간 발행
        self._patrol = None        # {'task_id', 'wps', 'seg', 'seg_size'}
        self.patrol_done = False   # 마지막 구간까지 완료 여부(검증용)
        self.last_waypoint_id = -1  # 마지막 구간 result의 last_waypoint_id(검증용)
        self.capture_ids = []      # 이번 순찰의 촬영 지점(capture=true) 목록(검증용)

        # E0 FleetTelemetry 구독
        self.create_subscription(
            FleetTelemetry, '/automato/telemetry/fleet',
            self._on_fleet, 10, callback_group=self._cb)

        # E1 Navigate 액션 클라이언트
        self._navigate_cli = ActionClient(
            self, Navigate, '/%s/navigate' % self.robot_id, callback_group=self._cb)

        # E2 SaveDetection 서비스 서버
        self.create_service(
            SaveDetection, '/automato/save_detection',
            self._on_save_detection, callback_group=self._cb)

        # 수동 트리거
        self.create_service(
            Trigger, '/acs_sim/start_patrol', self._on_trigger, callback_group=self._cb)

        self.get_logger().info('ACS 시뮬 시작: Navigate클라 /%s/navigate, SaveDetection서버, Fleet구독'
                               % self.robot_id)

        if self.get_parameter('auto_start').value:
            delay = float(self.get_parameter('auto_delay').value)
            self.create_timer(delay, self._auto_start_once, callback_group=self._cb)

    # ---- E0 ----
    def _on_fleet(self, msg):
        self.fleet_count += 1
        self.last_fleet = msg
        self.get_logger().info('Fleet 수신: ddago=%d대 ddagi=%d대'
                               % (len(msg.ddagos), len(msg.ddagis)))

    # ---- E2 SaveDetection 서버 ----
    def _on_save_detection(self, request, response):
        has_image = request.disease_image.height > 0 and request.disease_image.width > 0
        self.saved.append({
            'task_id': request.task_id, 'waypoint_id': request.waypoint_id,
            'ripe': request.ripe_percent, 'unripe': request.unripe_percent,
            'rotten': request.rotten_percent, 'disease': request.disease_percent,
            'has_image': has_image})
        self.get_logger().info(
            'SaveDetection 저장: task=%d wp=%d ripe=%d unripe=%d rotten=%d disease=%d image=%s'
            % (request.task_id, request.waypoint_id, request.ripe_percent,
               request.unripe_percent, request.rotten_percent, request.disease_percent,
               ('%dx%d' % (request.disease_image.width, request.disease_image.height)) if has_image else '없음'))
        if has_image:
            self._save_image(request)
        response.success = True
        response.message = '저장 완료(sim)'
        return response

    def _save_image(self, request):
        """SaveDetection 으로 받은 sensor_msgs/Image(rgb8) 를 파일로 저장(수신 확인용)."""
        try:
            from PIL import Image as PILImage
            im = PILImage.frombytes('RGB', (request.disease_image.width, request.disease_image.height),
                                    bytes(request.disease_image.data))
            os.makedirs(ACS_SAVE_DIR, exist_ok=True)
            path = os.path.join(ACS_SAVE_DIR, 'save_task%d_wp%d.jpg'
                                % (request.task_id, request.waypoint_id))
            im.save(path)
            self.get_logger().info('SaveDetection 이미지 저장: %s (%dx%d)'
                                   % (path, im.width, im.height))
        except Exception as e:   # noqa: BLE001
            self.get_logger().warn('SaveDetection 이미지 저장 실패: %s' % e)

    # ---- E1 순찰 발행 ----
    def _auto_start_once(self):
        # 타이머는 1회만 쓰기 위해 즉시 취소
        for t in list(self.timers):
            t.cancel()
        self.send_patrol()

    def _on_trigger(self, request, response):
        task_id = self.send_patrol()
        response.success = True
        response.message = '순찰 발행 task_id=%d' % task_id
        return response

    def send_patrol(self, num_waypoints=None, seg_size=None):
        """순찰 경로를 만들어 첫 구간(Waypoint[])을 하달. 이후 구간 result마다 다음 구간을 하달.

        capture 규칙(시뮬): 홀수 waypoint_id = 순찰 지점(capture=true, 촬영·분석),
        짝수 = 통과 노드(capture=false). 실제 ACS 는 예약 결과에 따라 정한다."""
        if self._patrol is not None:
            # 실제 ACS 는 진행 중 task 가 있는 로봇을 배정하지 않는다(unavailable_reason=ROBOT_BUSY).
            self.get_logger().warn('ROBOT_BUSY — 진행 중인 task=%d 있음, 새 순찰 발행 안 함'
                                   % self._patrol['task_id'])
            return self._patrol['task_id']
        if num_waypoints is None:
            num_waypoints = int(self.get_parameter('num_waypoints').value)
        if seg_size is None:
            seg_size = int(self.get_parameter('seg_size').value)
        self._task_seq += 1
        task_id = self._task_seq
        wps = []
        for i in range(num_waypoints):
            wp = Waypoint()
            wp.waypoint_id = i
            wp.x = float(i + 1)          # 첫 waypoint가 (0,0)이 되지 않도록 1부터
            wp.y = float(i + 1) * 0.5
            wp.capture = (i % 2 == 1)    # 순찰 지점에서만 촬영
            wps.append(wp)

        if not self._navigate_cli.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('DCS Navigate 서버 없음 — 발행 취소')
            return task_id
        self._patrol = {'task_id': task_id, 'wps': wps, 'seg': 0, 'seg_size': max(1, seg_size)}
        self.patrol_done = False
        self.last_waypoint_id = -1
        self.capture_ids = [w.waypoint_id for w in wps if w.capture]
        self.get_logger().info('순찰 시작: task_id=%d waypoints=%d 구간크기=%d 촬영지점=%s'
                               % (task_id, num_waypoints, seg_size, self.capture_ids))
        self._send_next_segment()
        return task_id

    def _send_next_segment(self):
        """예약 확보된 구간(seg_size 개)만큼 잘라서 하달 (E2 20번 흉내)."""
        p = self._patrol
        if p is None:
            return
        start = p['seg'] * p['seg_size']
        seg = p['wps'][start:start + p['seg_size']]
        if not seg:
            return
        goal = Navigate.Goal(task_id=p['task_id'], waypoints=seg)
        self.get_logger().info('구간 하달: task=%d waypoints=%s (구간 %d)'
                               % (p['task_id'], [w.waypoint_id for w in seg], p['seg'] + 1))
        fut = self._navigate_cli.send_goal_async(goal, feedback_callback=self._on_navigate_fb)
        fut.add_done_callback(self._on_navigate_goal_response)

    def _on_navigate_goal_response(self, future):
        gh = future.result()
        if not gh.accepted:
            self.get_logger().error('DCS가 Navigate goal 거부')
            self._patrol = None
            return
        gh.get_result_async().add_done_callback(self._on_navigate_result)

    def _on_navigate_fb(self, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().info('순찰 진행: wp=%d idx=%d (%.2f,%.2f)'
                               % (fb.current_waypoint_id, fb.waypoint_index,
                                  fb.current_x, fb.current_y))

    def _on_navigate_result(self, future):
        res = future.result().result
        self.last_result = res
        self.last_waypoint_id = res.last_waypoint_id
        p = self._patrol
        if p is None:
            return
        self.get_logger().info('구간 결과: code=%d last_wp=%d msg=%s'
                               % (res.result_code, res.last_waypoint_id, res.message))
        p['seg'] += 1
        remaining = len(p['wps']) - p['seg'] * p['seg_size']
        if res.result_code == 0 and remaining > 0:
            self._send_next_segment()   # 재계획 후 다음 구간 하달 (E2 18~20번)
        else:
            self.patrol_done = True
            self.get_logger().info('순찰 완료: task=%d last_wp=%d code=%d'
                                   % (p['task_id'], res.last_waypoint_id, res.result_code))
            self._patrol = None


def main(args=None):
    rclpy.init(args=args)
    node = AcsSim()
    executor = MultiThreadedExecutor(num_threads=4)
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
