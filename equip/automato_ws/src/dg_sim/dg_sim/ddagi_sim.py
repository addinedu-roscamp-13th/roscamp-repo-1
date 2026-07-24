#!/usr/bin/env python3
"""Ddagi(로봇팔) Control Service 시뮬레이터.

담당:
  E0-2  DdagiTelemetry 1Hz 발행                   /ddagi/telemetry
        - 상시 발행하지 않고 **실행(트리거) 시에만**. auto_telemetry=true 면 상시.
  E3~E5 Harvest 액션 서버 (DCS ← )                /ddagi/harvest
        - 실제 수확 루프(관측·DetectTomatoes·제외목록·파지) 대신, 라운드별 Feedback 을
          순서대로 흘리고 종료 사유별 Result 를 돌려 **DCS 의 중계**(feedback·result·cancel)
          와 **무수신 워치독**을 검증한다. 검출·좌표는 이 시뮬 범위 밖(내부 흉내).
        - harvest_mode 로 종료 사유를 주입: depleted(DEPLETED) / full(FULL) /
          max_rounds(MAX_ROUNDS_EXCEEDED) / hang(무응답 → DCS watchdog). 테스트는
          self.harvest_mode 를 바꿔 케이스를 재사용한다.

Topic: /ddagi/telemetry (automato_interfaces/msg/DdagiTelemetry, 1Hz, robot_id 는 메시지 필드)
"""
import time

import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger

from automato_interfaces.action import Harvest
from automato_interfaces.msg import DdagiTelemetry, ServoStatus


class DdagiSim(Node):
    def __init__(self, **kwargs):
        super().__init__('ddagi_sim', **kwargs)
        self.declare_parameter('robot_id', 'dg_01')
        self.declare_parameter('auto_telemetry', False)   # 상시 발행 여부(기본 off)
        self.declare_parameter('burst_sec', 8.0)          # 트리거 시 발행 지속(초)
        # 수확 시뮬 파라미터
        self.declare_parameter('harvest_mode', 'depleted')     # depleted/full/max_rounds/hang
        self.declare_parameter('harvest_rounds', 2)            # 라운드 수
        self.declare_parameter('harvest_picks_per_round', 3)   # 라운드당 파지 수
        self.declare_parameter('harvest_step_delay', 0.2)      # 파지 1회 처리 시간(초)
        self.robot_id = self.get_parameter('robot_id').value
        self.burst_sec = float(self.get_parameter('burst_sec').value)
        self.harvest_mode = self.get_parameter('harvest_mode').value
        self.harvest_step_delay = float(self.get_parameter('harvest_step_delay').value)
        self._cb = ReentrantCallbackGroup()
        self._task_id = 1024
        self._tel_until = float('inf') if self.get_parameter('auto_telemetry').value else 0.0
        self._pub = self.create_publisher(
            DdagiTelemetry, '/ddagi/telemetry', 10)
        self.create_timer(1.0, self._tick, callback_group=self._cb)
        self.create_service(Trigger, '/ddagi_sim/start_telemetry', self._on_start_tel,
                            callback_group=self._cb)
        self.create_service(Trigger, '/ddagi_sim/stop_telemetry', self._on_stop_tel,
                            callback_group=self._cb)
        # E3~E5 Harvest 액션 서버 (DCS ← )
        self._harvest_srv = ActionServer(
            self, Harvest, '/ddagi/harvest',   # 연동에 robot_id 미사용
            execute_callback=self._harvest_execute,
            cancel_callback=lambda _gh: CancelResponse.ACCEPT,
            callback_group=self._cb)
        # 라이브에서 `ros2 param set /ddagi_sim harvest_mode full` 로 종료 사유 주입.
        self.add_on_set_parameters_callback(self._on_set_params)
        self.get_logger().info('Ddagi 시뮬 시작 → /ddagi/{telemetry,harvest} (harvest_mode=%s)'
                               % self.harvest_mode)

    VALID_HARVEST_MODES = ('depleted', 'full', 'max_rounds', 'hang')

    def _on_set_params(self, params):
        for p in params:
            if p.name == 'harvest_mode':
                if p.value not in self.VALID_HARVEST_MODES:
                    return SetParametersResult(
                        successful=False,
                        reason='harvest_mode 는 %s 중 하나' % ', '.join(self.VALID_HARVEST_MODES))
                self.harvest_mode = p.value
                self.get_logger().info('harvest_mode 변경 → %s' % p.value)
        return SetParametersResult(successful=True)

    def _on_start_tel(self, request, response):
        self._tel_until = float('inf')   # 중지 전까지 상시 발행
        self.get_logger().info('Ddagi 텔레메트리 발행 시작(상시)')
        response.success = True
        response.message = 'ddagi telemetry started'
        return response

    def _on_stop_tel(self, request, response):
        self._tel_until = 0.0
        self.get_logger().info('Ddagi 텔레메트리 발행 중지')
        response.success = True
        response.message = 'ddagi telemetry stopped'
        return response

    def _tick(self):
        if time.time() > self._tel_until:
            return   # 실행 트리거 전/후에는 발행 안 함
        msg = DdagiTelemetry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.robot_id = self.robot_id
        msg.task_id = self._task_id
        msg.is_paused = False
        msg.joint_angles = [10.2, -30.5, 45.0, 0.0, -12.3, 5.5]
        msg.tcp_coords = [160.0, 30.0, 200.0, 0.0, 0.0, 0.0]
        servos = []
        for j in range(1, 8):
            s = ServoStatus()
            s.joint_no = j
            s.voltage_ok = True
            s.temperature = 40 - j
            s.current = 0.5
            s.overload = False
            s.gripper_value = 0 if j != 7 else 100
            servos.append(s)
        msg.servo_health = servos
        self._pub.publish(msg)

    # ---- E3~E5 Harvest (DCS ← ) : 라운드 Feedback + 종료 사유별 Result ----
    @staticmethod
    def _harvest_result(normal, discard, failed, exit_reason, message):
        r = Harvest.Result()
        r.normal_count = normal
        r.discard_count = discard
        r.failed_count = failed
        r.exit_reason = exit_reason
        r.message = message
        return r

    def _harvest_execute(self, goal_handle):
        req = goal_handle.request
        mode = self.harvest_mode
        self.get_logger().info('수확 goal 수신: task=%d max_capacity=%d mode=%s'
                               % (req.task_id, req.max_capacity, mode))
        rounds = int(self.get_parameter('harvest_rounds').value)
        picks = int(self.get_parameter('harvest_picks_per_round').value)
        delay = self.harvest_step_delay   # 테스트가 직접 바꿀 수 있게 속성 사용

        # hang: 진행 소식(Feedback) 없이 대기 → DCS 의 무수신 워치독 검증용.
        if mode == 'hang':
            self.get_logger().warn('수확 무응답 시뮬(hang) — Feedback 없이 대기')
            waited = 0.0
            while rclpy.ok() and not goal_handle.is_cancel_requested and waited < 6.0:
                time.sleep(0.1)
                waited += 0.1
            r = self._harvest_result(0, 0, 0, '', 'hang 종료')
            if not rclpy.ok():
                return r
            try:
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    r.message = 'hang 취소'
                else:
                    goal_handle.abort()
            except Exception:   # noqa: BLE001 - teardown 경합 시 무시
                pass
            return r

        normal = discard = failed = 0
        for rnd in range(1, rounds + 1):
            remaining = picks
            for _ in range(picks):
                if goal_handle.is_cancel_requested:
                    self.get_logger().warn('취소 요청 → 수확 중단 (round=%d)' % rnd)
                    goal_handle.canceled()
                    return self._harvest_result(normal, discard, failed, '', '취소로 중단')
                fb = Harvest.Feedback()
                fb.round = rnd
                fb.normal_count = normal
                fb.discard_count = discard
                fb.failed_count = failed
                fb.remaining_in_round = remaining
                goal_handle.publish_feedback(fb)
                if delay > 0:
                    time.sleep(delay)
                normal += 1              # 파지 1회 결과 흉내(대체로 NORMAL 적재)
                remaining -= 1
                # full: 수확품 바구니 만차 도달 시 즉시 종료
                if mode == 'full' and normal >= req.max_capacity:
                    goal_handle.succeed()
                    return self._harvest_result(normal, discard, failed, 'FULL', '만차(시뮬)')

        goal_handle.succeed()
        if mode == 'max_rounds':
            return self._harvest_result(normal, discard, failed,
                                        'MAX_ROUNDS_EXCEEDED', '라운드 상한(시뮬)')
        return self._harvest_result(normal, discard, failed, 'DEPLETED', '대상 소진(시뮬)')


def main(args=None):
    rclpy.init(args=args)
    node = DdagiSim()
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
