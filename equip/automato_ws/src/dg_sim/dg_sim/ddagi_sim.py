#!/usr/bin/env python3
"""Ddagi(로봇팔) Control Service 시뮬레이터.

시퀀스 다이어그램 E0-2: DdagiTelemetry 를 1Hz 로 발행.
단, 시뮬에서는 상시 발행하지 않고 **실행(트리거) 시에만** burst_sec 동안 발행한다.
  - 서비스 /ddagi_sim/start_telemetry (std_srvs/Trigger) 호출 시 burst_sec 초 동안 1Hz 발행
  - 파라미터 auto_telemetry=true 면 상시 발행(테스트/상시 필요 시)

Topic: /ddagi/telemetry (automato_interfaces/msg/DdagiTelemetry, 1Hz, robot_id 는 메시지 필드)
"""
import time

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

from automato_interfaces.msg import DdagiTelemetry, ServoStatus


class DdagiSim(Node):
    def __init__(self, **kwargs):
        super().__init__('ddagi_sim', **kwargs)
        self.declare_parameter('robot_id', 'dg_01')
        self.declare_parameter('auto_telemetry', False)   # 상시 발행 여부(기본 off)
        self.declare_parameter('burst_sec', 8.0)          # 트리거 시 발행 지속(초)
        self.robot_id = self.get_parameter('robot_id').value
        self.burst_sec = float(self.get_parameter('burst_sec').value)
        self._task_id = 1024
        self._tel_until = float('inf') if self.get_parameter('auto_telemetry').value else 0.0
        self._pub = self.create_publisher(
            DdagiTelemetry, '/ddagi/telemetry', 10)
        self.create_timer(1.0, self._tick)
        self.create_service(Trigger, '/ddagi_sim/start_telemetry', self._on_start_tel)
        self.create_service(Trigger, '/ddagi_sim/stop_telemetry', self._on_stop_tel)
        self.get_logger().info('Ddagi 시뮬 시작 → /ddagi/telemetry (실행 시 연속 발행, 중지까지)')

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


def main(args=None):
    rclpy.init(args=args)
    node = DdagiSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
