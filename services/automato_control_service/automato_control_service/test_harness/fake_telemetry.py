#!/usr/bin/env python3
"""RP-78 테스트 스탠드인 ③ — 가짜 로봇 텔레메트리 발행기 (물리 로봇 없이 로봇 1대를 흉내).

⚠️ 실제 로봇도, 실제 DG Control Service 도 아니다. A 티어(로직 검증)에서 물리 로봇 없이
   "가용한 로봇이 하나 더 있는 것"처럼 보이게 하는 '테스트 전용' 노드다.
   진짜 로봇/HQ 가 준비되면 이 파일은 버린다. (fleet_aggregator / patrol_bridge 와 같은 성격)

왜 필요한가 —
   ACS(RP-78)의 로봇 선정·교통관제는 "가용한 로봇들의 상태"를 보고 판단한다. 그 상태를 올려주는
   건 원래 각 로봇의 ddago_telemetry 인데, 물리 로봇이 1대뿐이면
     * T3 '여럿 중 배터리 최고 고르기'
     * T7 '두 로봇이 같은 통로 경합 → 양보'
   같은 다중 로봇 시나리오를 만들 수 없다. 이 노드가 로봇 한 대가 내는 것과 '똑같은'
   DdagoTelemetry 를 대신 발행해, 물리 로봇 없이 가짜 로봇(dg_02, dg_03 ...)을 만든다.

하는 일 —
   발행: /<robot_id>/ddago/telemetry  (DdagoTelemetry, 기본 1Hz)
     * 진짜 로봇과 같은 토픽/타입 → fleet_aggregator 가 구분 없이 취합해 ACS 로 전달.
     * header.stamp 를 매번 '지금'(시스템 시각)으로 찍는다 → 계속 신선.
       멈추면(Ctrl+C) 3초 뒤 ACS 에서 자연히 TELEMETRY_STALE 로 뜬다.
   값(배터리·상태·좌표 등)은 파라미터로 주고 매 틱마다 다시 읽어 발행하므로, 실행 중에
   `ros2 param set` 으로 바꾸면 '즉시' 반영된다(재시작 불필요). 이게 T2/T3 재현의 핵심:
     * 저배터리 → BATTERY_TOO_LOW :  ros2 param set /dg_02/fake_telemetry battery_percent 65.0
     * 주행중  → ROBOT_BUSY       :  ros2 param set /dg_02/fake_telemetry nav_status NAVIGATING
     * 다시 가용                  :  ros2 param set /dg_02/fake_telemetry nav_status IDLE
     * 미수신  → TELEMETRY_STALE  :  이 노드를 Ctrl+C 로 멈춤

실행 (🖥️ 관제 PC, ROS + automato_interfaces 소싱 후) — 가짜 로봇 1대당 하나씩:
   # robot_id 는 네임스페이스(-r __ns:=/dg_02)에서 자동 유도된다(생략 가능).
   python3 -m automato_control_service.test_harness.fake_telemetry \
       --ros-args -r __ns:=/dg_02 -p battery_percent:=90.0
   # (패키지를 colcon 빌드했다면) ros2 run 으로도:
   #   ros2 run automato_control_service fake_telemetry --ros-args -r __ns:=/dg_02

주의 —
   * fleet_aggregator 의 robot_ids 에 이 가짜 robot_id 가 포함돼야 취합된다
     (예: 진짜 dg_01 + 가짜 dg_02 → robot_ids:="['dg_01','dg_02']").
   * 같은 robot_id 를 진짜 로봇과 가짜가 동시에 내면 안 된다(같은 토픽에 두 발행자 → 충돌).
     진짜 dg_01 + 가짜 dg_02 처럼 서로 다른 id 로 쓴다.
   * 값을 param set 으로 바꿀 땐 선언된 타입에 맞춰라(소수 파라미터는 65.0 처럼 소수점 포함).
"""
from automato_interfaces.msg import DdagoTelemetry
import rclpy
from rclpy.node import Node


class FakeTelemetry(Node):
    def __init__(self, **kwargs):
        super().__init__("fake_telemetry", **kwargs)

        # robot_id: 파라미터가 있으면 그것, 없으면 네임스페이스(-r __ns:=/dg_02)에서 유도.
        self.declare_parameter("robot_id", "")
        rid = (self.get_parameter("robot_id").value
               or self.get_namespace().strip("/") or "dg_02")
        self._robot_id = rid

        # 아래는 매 틱 다시 읽는 '실시간 변경' 대상 — ros2 param set 으로 바꾸면 즉시 반영.
        self.declare_parameter("nav_status", "IDLE")       # 'IDLE' 이어야 가용 후보
        self.declare_parameter("battery_percent", 90.0)    # 임계값(기본 70) 이상이면 가용
        self.declare_parameter("battery_voltage", 12.4)
        self.declare_parameter("x", 1.0)
        self.declare_parameter("y", 1.0)
        self.declare_parameter("yaw", 0.5)
        self.declare_parameter("us_range_m", 1.5)
        self.declare_parameter("task_id", 0)
        self.declare_parameter("is_charging", False)       # 판정엔 영향 없음(스냅샷용)
        self.declare_parameter("publish_rate_hz", 1.0)

        rate = float(self.get_parameter("publish_rate_hz").value)

        # 진짜 로봇과 '같은 토픽/타입' 으로 발행 → fleet_aggregator 가 구분 없이 취합.
        # 절대 토픽으로 만들어, 네임스페이스를 깜빡해도 경로가 어긋나지 않게 한다.
        self._pub = self.create_publisher(
            DdagoTelemetry, f"/{rid}/ddago/telemetry", 10)
        self.create_timer(1.0 / rate if rate > 0.0 else 1.0, self._tick)

        node_fqn = f"{self.get_namespace().rstrip('/')}/fake_telemetry"
        self.get_logger().info(
            f"[TEST] 가짜 텔레메트리: robot_id={rid} → /{rid}/ddago/telemetry "
            f"({rate:.1f}Hz). ※ 실제 로봇/HQ 아님(테스트 스탠드인). "
            f"값 변경 예) ros2 param set {node_fqn} battery_percent 65.0")

    def _tick(self) -> None:
        g = self.get_parameter
        msg = DdagoTelemetry()
        # header.stamp = '지금'(시스템 시각). ACS staleness(3초)를 통과 → 멈추면 자연히 STALE.
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._robot_id
        msg.robot_id = self._robot_id                      # aggregator/ACS 가 이 값으로 키잉
        msg.task_id = int(g("task_id").value)
        msg.nav_status = str(g("nav_status").value)
        msg.is_charging = bool(g("is_charging").value)
        msg.x = float(g("x").value)
        msg.y = float(g("y").value)
        msg.yaw = float(g("yaw").value)
        msg.battery_percent = float(g("battery_percent").value)
        msg.battery_voltage = float(g("battery_voltage").value)
        msg.us_range_m = float(g("us_range_m").value)
        self._pub.publish(msg)
        self.get_logger().info(
            f"[TEST] {self._robot_id}: nav={msg.nav_status} "
            f"batt={msg.battery_percent:.0f}% pos=({msg.x:.2f},{msg.y:.2f})",
            throttle_duration_sec=5.0)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FakeTelemetry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
