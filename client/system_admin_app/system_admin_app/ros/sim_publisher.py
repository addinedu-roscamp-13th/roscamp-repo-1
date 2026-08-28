"""개발/시연용 모의 텔레메트리 발행기.

실제 로봇/HQ 없이 /automato/dashboard/fleet_telemetry(FleetTelemetry)를 1Hz로 발행한다.
- dg_01, dg_02 : 주행 + 로봇팔
- dg_03        : 주행 전용 (로봇팔 없음)
- dg_02        : 시연용으로 서보 과열/과부하가 주기적으로 발생 → '점검 필요' 뱃지 시연
공식 automato_interfaces 타입을 그대로 사용하므로, 실제 로봇 발행기와 상호 교체 가능.
"""
from __future__ import annotations

import math
import os
import time

import rclpy
from rclpy.node import Node

from automato_interfaces.msg import (
    FleetTelemetry, FleetMember, RobotTelemetry,
    DdagoTelemetry, DdagiTelemetry, ServoStatus,
)

DRIVE_ROBOTS = ["dg_01", "dg_02", "dg_03"]
ARM_ROBOTS = ["dg_01", "dg_02"]  # dg_03은 주행 전용

# 통신 두절 표시를 시험하기 위한 옵션. 이 로봇은 stamp를 갱신하지 않아
# (배열에는 계속 실리지만) QT에서 3초 뒤 '통신 두절'로 뜬다 — ACS의 실제 동작과 동일.
STALE_ROBOT = os.environ.get("AUTOMATO_SIM_STALE_ROBOT", "")


class SimPublisher(Node):
    def __init__(self):
        super().__init__("automato_sim_publisher")
        self.pub = self.create_publisher(
            FleetTelemetry, "/automato/dashboard/fleet_telemetry", 10
        )
        self.t0 = time.monotonic()
        self._frozen = None
        self.timer = self.create_timer(1.0, self._tick)  # 1Hz
        self.get_logger().info("모의 텔레메트리 발행 시작 (1Hz)")
        if STALE_ROBOT:
            self.get_logger().info(
                f"두절 시험: {STALE_ROBOT}의 stamp를 고정한다 "
                f"(배열에는 계속 실리지만 QT에서 통신 두절로 표시되어야 정상)"
            )

    def _now_msg_header(self):
        from std_msgs.msg import Header
        h = Header()
        h.stamp = self.get_clock().now().to_msg()
        h.frame_id = "map"
        return h

    def _frozen_header(self):
        """기동 시각에 고정된 header. 두절 로봇 흉내용(나이가 계속 늘어난다)."""
        if self._frozen is None:
            self._frozen = self._now_msg_header()
        return self._frozen

    def _tick(self):
        t = time.monotonic() - self.t0
        msg = FleetTelemetry()
        msg.header = self._now_msg_header()          # 취합 시각 (ACS 기준)

        # RP-114 재정의: 로봇별 FleetMember로 담는다. robot_id는 여기에만 있고,
        # 로봇 쪽 메시지(Ddago/Ddagi)의 robot_id는 삭제 예정이라 채우지 않는다.
        members: dict[str, FleetMember] = {}

        def member_of(robot_id: str) -> FleetMember:
            m = members.get(robot_id)
            if m is None:
                m = FleetMember()
                m.robot_id = robot_id
                m.telemetry = RobotTelemetry()
                # 두절 시험 대상은 stamp를 처음 값에 고정해 나이가 계속 늘어나게 한다.
                m.telemetry.header = (
                    self._frozen_header() if robot_id == STALE_ROBOT
                    else self._now_msg_header()
                )
                members[robot_id] = m
            return m

        for i, rid in enumerate(DRIVE_ROBOTS):
            d = DdagoTelemetry()
            d.header = self._now_msg_header()
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
            # us_range_m(초음파)은 QT 모니터링에서 제외됨 — 채우지 않는다(기본 0).
            member_of(rid).telemetry.ddagos.append(d)

        for i, rid in enumerate(ARM_ROBOTS):
            a = DdagiTelemetry()
            a.header = self._now_msg_header()
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
            member_of(rid).telemetry.ddagis.append(a)

        # ROBOT_IDS 순서를 유지해 화면에서 카드 순서가 흔들리지 않게 한다.
        msg.robots = [members[r] for r in DRIVE_ROBOTS if r in members]
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
