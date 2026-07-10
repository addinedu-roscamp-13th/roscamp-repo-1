#!/usr/bin/env python3
"""Automato Control Service 시뮬레이터.

팀원이 개발 중인 실제 Automato Control Service(ACS) 대역. 즉시-응답 스텁.

담당(시퀀스 다이어그램):
  E0  FleetTelemetry 구독 (HQ ← )                /automato/telemetry/fleet   (로그로 확인)
  E1  Patrol 액션 클라이언트 (→ HQ)              /{robot_id}/patrol
        - waypoint를 하나씩 하달(task_id, 단일 waypoint) → 결과 받으면 다음 것 (루프 주체=ACS)
  E2  SaveDetection 서비스 서버 (HQ ← )          /automato/save_detection    (즉시 success)

편의:
  - 파라미터 auto_start(기본 true)면 기동 후 auto_delay 초에 순찰 1회 자동 발행.
  - 서비스 /acs_sim/start_patrol (std_srvs/Trigger) 호출로 언제든 순찰 재발행.
"""
import os
import threading

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from automato_interfaces.action import Patrol
from automato_interfaces.msg import FleetTelemetry, WaypointGoal
from automato_interfaces.srv import SaveDetection

# SaveDetection 으로 받은 라벨 이미지 저장 폴더
ACS_SAVE_DIR = os.environ.get(
    'ACS_SIM_SAVE_DIR',
    '/home/ane/dev_ws/roscamp-sprint4-heeseog/equip/automato_ws/acs_recv')
from std_srvs.srv import Trigger


class AcsSim(Node):
    def __init__(self, **kwargs):
        super().__init__('acs_sim', **kwargs)
        self.declare_parameter('robot_id', 'dg_01')
        self.declare_parameter('auto_start', True)
        self.declare_parameter('auto_delay', 3.0)
        self.declare_parameter('num_waypoints', 4)
        self.robot_id = self.get_parameter('robot_id').value
        self._cb = ReentrantCallbackGroup()
        self._task_seq = 1024
        self.saved = []   # 수신한 SaveDetection 누적(검증/디버깅용)
        self.fleet_count = 0
        self.last_fleet = None
        self.last_result = None
        # 순찰 루프 상태: waypoint를 하나씩 하달하고 결과를 받으면 다음 것 발행
        self._patrol = None        # {'task_id', 'wps', 'idx', 'visited'}
        self.patrol_done = False   # 마지막 waypoint까지 완료 여부(검증용)
        self.last_visited = 0      # 마지막 순찰의 방문 수(검증용)

        # E0 FleetTelemetry 구독
        self.create_subscription(
            FleetTelemetry, '/automato/telemetry/fleet',
            self._on_fleet, 10, callback_group=self._cb)

        # E1 Patrol 액션 클라이언트
        self._patrol_cli = ActionClient(
            self, Patrol, '/%s/patrol' % self.robot_id, callback_group=self._cb)

        # E2 SaveDetection 서비스 서버
        self.create_service(
            SaveDetection, '/automato/save_detection',
            self._on_save_detection, callback_group=self._cb)

        # 수동 트리거
        self.create_service(
            Trigger, '/acs_sim/start_patrol', self._on_trigger, callback_group=self._cb)

        self.get_logger().info('ACS 시뮬 시작: Patrol클라 /%s/patrol, SaveDetection서버, Fleet구독'
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

    def send_patrol(self, num_waypoints=None):
        """순찰 경로(waypoint 목록)를 만들어 첫 waypoint를 하달. 이후 결과마다 다음 것을 이어서 발행."""
        if num_waypoints is None:
            num_waypoints = int(self.get_parameter('num_waypoints').value)
        self._task_seq += 1
        task_id = self._task_seq
        wps = []
        for i in range(num_waypoints):
            wp = WaypointGoal()
            wp.waypoint_id = i
            wp.x = float(i + 1)          # 첫 waypoint가 (0,0)이 되지 않도록 1부터
            wp.y = float(i + 1) * 0.5
            wps.append(wp)

        if not self._patrol_cli.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('HQ Patrol 서버 없음 — 발행 취소')
            return task_id
        self._patrol = {'task_id': task_id, 'wps': wps, 'idx': 0, 'visited': 0}
        self.patrol_done = False
        self.get_logger().info('순찰 시작: task_id=%d waypoints=%d' % (task_id, num_waypoints))
        self._send_next_waypoint()
        return task_id

    def _send_next_waypoint(self):
        p = self._patrol
        if p is None or p['idx'] >= len(p['wps']):
            return
        wp = p['wps'][p['idx']]
        goal = Patrol.Goal(task_id=p['task_id'], waypoint=wp)
        self.get_logger().info('waypoint 하달: task=%d wp=%d (%.2f,%.2f) [%d/%d]'
                               % (p['task_id'], wp.waypoint_id, wp.x, wp.y,
                                  p['idx'] + 1, len(p['wps'])))
        fut = self._patrol_cli.send_goal_async(goal, feedback_callback=self._on_patrol_fb)
        fut.add_done_callback(self._on_patrol_goal_response)

    def _on_patrol_goal_response(self, future):
        gh = future.result()
        if not gh.accepted:
            self.get_logger().error('HQ가 Patrol goal 거부')
            self._patrol = None
            return
        gh.get_result_async().add_done_callback(self._on_patrol_result)

    def _on_patrol_fb(self, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().info('순찰 진행: wp=%d (%.2f,%.2f)'
                               % (fb.current_waypoint_id, fb.current_x, fb.current_y))

    def _on_patrol_result(self, future):
        res = future.result().result
        self.last_result = res
        p = self._patrol
        if p is None:
            return
        if res.result_code == 0:
            p['visited'] += 1
        self.get_logger().info('waypoint 결과: code=%d msg=%s [%d/%d]'
                               % (res.result_code, res.message, p['idx'] + 1, len(p['wps'])))
        p['idx'] += 1
        if res.result_code == 0 and p['idx'] < len(p['wps']):
            self._send_next_waypoint()          # 다음 waypoint 이어서 하달
        else:
            self.last_visited = p['visited']
            self.patrol_done = True
            self.get_logger().info('순찰 완료: task=%d visited=%d/%d'
                                   % (p['task_id'], p['visited'], len(p['wps'])))
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
