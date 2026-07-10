"""개발/시연용 모의 텔레메트리 발행기.

실제 로봇/HQ 없이 /automato/dashboard/fleet_telemetry(FleetTelemetry)를 1Hz로 발행한다.
- dg_01, dg_02 : 주행 + 로봇팔
- dg_03        : 주행 전용 (로봇팔 없음)
- dg_02        : 시연용으로 서보 과열/과부하가 주기적으로 발생 → '점검 필요' 뱃지 시연
공식 automato_interfaces 타입을 그대로 사용하므로, 실제 로봇 발행기와 상호 교체 가능.
"""
from __future__ import annotations

import math
import time

import rclpy
from rclpy.node import Node

from automato_interfaces.msg import (
    FleetTelemetry, DdagoTelemetry, DdagiTelemetry, ServoStatus,
)

DRIVE_ROBOTS = ["dg_01", "dg_02", "dg_03"]
ARM_ROBOTS = ["dg_01", "dg_02"]  # dg_03은 주행 전용


class SimPublisher(Node):
    def __init__(self):
        super().__init__("automato_sim_publisher")
        self.pub = self.create_publisher(
            FleetTelemetry, "/automato/dashboard/fleet_telemetry", 10
        )
        self.t0 = time.monotonic()
        self.timer = self.create_timer(1.0, self._tick)  # 1Hz
        self.get_logger().info("모의 텔레메트리 발행 시작 (1Hz)")

    def _now_msg_header(self):
        from std_msgs.msg import Header
        h = Header()
        h.stamp = self.get_clock().now().to_msg()
        h.frame_id = "map"
        return h

    def _tick(self):
        t = time.monotonic() - self.t0
        msg = FleetTelemetry()
        msg.header = self._now_msg_header()

        for i, rid in enumerate(DRIVE_ROBOTS):
            d = DdagoTelemetry()
            d.header = self._now_msg_header()
            d.robot_id = rid
            d.task_id = 1024 + i
            d.nav_status = "PATROLLING" if i == 0 else ("NAVIGATING" if i == 1 else "IDLE")
            d.is_charging = False
            # automato_map 범위(x∈[-0.19,0.98], y∈[-0.63,1.29]) 안에서 움직이도록
            cx, cy = 0.40, 0.33
            d.x = cx + 0.22 * math.sin(t / 5 + i * 2.1)
            d.y = cy + 0.45 * math.cos(t / 5 + i * 2.1)
            d.yaw = math.sin(t / 8 + i)
            # 배터리는 로봇별로 서서히 감소
            d.battery_percent = max(15.0, 90.0 - i * 12 - (t * 0.05))
            d.battery_voltage = 10.5 + d.battery_percent * 0.02
            d.us_range_m = 0.25 + 0.2 * (1 + math.sin(t / 3 + i))
            msg.ddagos.append(d)

        for i, rid in enumerate(ARM_ROBOTS):
            a = DdagiTelemetry()
            a.header = self._now_msg_header()
            a.robot_id = rid
            a.task_id = 1024 + i
            a.is_paused = False
            a.joint_angles = [
                float(10 * math.sin(t / 4 + j)) for j in range(6)
            ]
            a.tcp_coords = [160.0, 30.0, 200.0, 0.0, 0.0, 0.0]

            servos = []
            # dg_02는 시연용으로 과열/과부하 유도
            hot = (rid == "dg_02") and (int(t) % 20 >= 10)
            for j in range(1, 8):
                s = ServoStatus()
                s.joint_no = j
                s.voltage_ok = True
                base = 38 + j
                s.temperature = base + (20 if hot else int(3 * math.sin(t / 2 + j)))
                s.current = 0.2 + 0.1 * (j % 3)
                s.overload = bool(hot and j == 3)
                s.gripper_value = 85 if j == 7 else 0
                servos.append(s)
            a.servo_health = servos
            msg.ddagis.append(a)

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SimPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
