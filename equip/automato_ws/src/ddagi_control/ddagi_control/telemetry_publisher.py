#!/usr/bin/env python3
"""시나리오1 E0  Ddagi(로봇팔) 텔레메트리 퍼블리셔 (Ddagi Control Service).

로봇팔의 헬스 상태(servo_health: 6관절 + 그리퍼)를 담은 DdagiTelemetry 를
1Hz 로 HQ(DG Control Service)에 퍼블리시한다. sprint4 범위는 헬스 상태
발행까지이며, task_id/순찰-수확 연동은 다루지 않는다(항상 유휴 상태로 발행).

실물 myCobot 연동 상태 — read_servo_health() 가 arm_hardware.get_arm() 의
같은 pymycobot 인스턴스를 조회해 헬스 상태를 채운다(시리얼 포트 단일 인스턴스
공유 원칙, docs/ddagi_selective_switching.md §2-3 참고).

주의: harvest_server(별도 프로세스)와 telemetry_publisher를 동시에 실행하면
각자 arm_hardware.get_arm()으로 별도 시리얼 연결을 열게 되어 포트 충돌이
날 수 있다 — 현재는 둘을 동시에 띄우지 않는 것으로 회피한다.

토픽: /{robot_id}/ddagi/telemetry  (문서 스펙 예: /dg_01/ddagi/telemetry)
메시지: automato_interfaces/msg/DdagiTelemetry

파라미터:
  robot_id         (string) 로봇 식별자        기본 'dg_01'
  publish_rate_hz  (double) 퍼블리시 주기(Hz)  기본 1.0
"""
import rclpy
from rclpy.node import Node

from automato_interfaces.msg import DdagiTelemetry, ServoStatus
from ddagi_control.arm_hardware import get_arm

JOINT_COUNT = 6
SERVO_COUNT = JOINT_COUNT + 1  # 6관절 + 그리퍼(7번)


def read_servo_health():
    """실물 서보 헬스 상태 조회 (6관절 + 그리퍼)."""
    arm = get_arm()
    temps = arm.get_servo_temps()
    voltages = arm.get_servo_voltages()
    errors = arm.get_servo_status()
    gripper_value = arm.get_gripper_value()

    health = []
    for i in range(JOINT_COUNT):
        status = ServoStatus()
        status.joint_no = i + 1
        status.voltage_ok = 0 < voltages[i] < 24
        status.temperature = temps[i]
        status.current = 0.0  # 벌크 조회 API 없음 — 후속 과제
        status.overload = bool(errors[i])
        status.gripper_value = 0
        health.append(status)

    gripper = ServoStatus()
    gripper.joint_no = 7
    gripper.voltage_ok = True
    gripper.temperature = 0
    gripper.current = 0.0
    gripper.overload = False
    gripper.gripper_value = gripper_value
    health.append(gripper)
    return health


class TelemetryPublisher(Node):
    def __init__(self):
        super().__init__('ddagi_telemetry_publisher')
        self.declare_parameter('robot_id', 'dg_01')
        self.declare_parameter('publish_rate_hz', 1.0)

        self._robot_id = self.get_parameter('robot_id').value
        rate_hz = self.get_parameter('publish_rate_hz').value

        topic = f'/{self._robot_id}/ddagi/telemetry'
        self._pub = self.create_publisher(DdagiTelemetry, topic, 10)
        self._timer = self.create_timer(1.0 / rate_hz, self._on_timer)
        self.get_logger().info(f'Ddagi 텔레메트리 퍼블리셔 시작: {topic} ({rate_hz}Hz)')

    def _on_timer(self):
        arm = get_arm()
        msg = DdagiTelemetry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.robot_id = self._robot_id
        # sprint4 범위 밖: 작업 연동 전이므로 유휴 상태 고정값 사용.
        msg.task_id = 0
        msg.is_paused = False
        msg.joint_angles = [float(v) for v in arm.get_angles()]
        msg.tcp_coords = [float(v) for v in arm.get_coords()]
        msg.servo_health = read_servo_health()
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TelemetryPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
