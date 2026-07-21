#!/usr/bin/env python3
"""RP-114 테스트 스탠드인 ① — DG Control Service 대역 (텔레메트리 중계 부분만).

⚠️ 실제 DG Control Service(dcs) 가 아니다. 로봇 텔레메트리를 받아 자기 세트분을
   /{robot_id}/telemetry 로 올려주는 최소 기능만 흉내내는 '테스트 전용' 노드다.

RP-78 때 이 파일은 fleet_aggregator 였다 — 로봇 3대분을 FleetTelemetry 하나로 묶어
/automato/telemetry/fleet 에 발행했다. RP-114 로 취합 책임이 ACS 로 넘어가면서 그 역할이
사라졌고, 남은 일은 '로봇 텔레메트리를 DG 경계로 올려보내기'뿐이라 이름과 출력이 바뀌었다.

실기와 다른 점 —
  실기 dcs 는 로봇 세트마다 한 프로세스씩 뜨고, 물리망이 분리돼 있어 자기 로봇만 본다
  (그래서 메시지 안에 robot_id 가 필요 없다). 이 대역은 한 머신에서 도는 테스트용이라
  물리 분리가 없다. 그래서 fake_telemetry 가 payload 에 실어주는 msg.robot_id 로 로봇을
  가르고, 한 프로세스가 DG 여러 개인 척 로봇별 토픽에 나눠 발행한다.

하는 일:
  구독:  /ddago/telemetry  (DdagoTelemetry)   ← 모든 로봇이 '한 토픽'에 발행, robot_id 로 구분
         /ddagi/telemetry  (DdagiTelemetry)   ← 팔 있는 로봇만(없으면 비어도 정상)
  발행:  /{robot_id}/telemetry  (RobotTelemetry, 기본 1Hz) — 로봇마다 하나씩

핵심: 받은 텔레메트리 '원본'을 그대로 담는다(특히 header.stamp 보존).
  ACS의 가용 판정 staleness 는 ddago header.stamp(로봇이 찍은 시각) 기준이라,
  중계 노드가 stamp 를 새로 찍으면 안 된다. 원본을 통과시켜야 정확하다.
  (RobotTelemetry.header 는 중계 시각으로 새로 찍는다 — 개별 로봇 stamp 와는 별개.)

실행 (🖥️ 관제 PC):
  # ACS 패키지가 colcon 빌드/소싱돼 있으면
  ros2 run automato_control_service dg_stub
  # 또는 모듈로 직접 (services/automato_control_service 에서, ROS 소싱 후)
  python3 -m automato_control_service.test_harness.dg_stub
"""
from automato_interfaces.msg import DdagiTelemetry, DdagoTelemetry, RobotTelemetry
import rclpy
from rclpy.node import Node

from automato_control_service.fleet_collector import robot_telemetry_topic


class DgStub(Node):
    def __init__(self, **kwargs):
        super().__init__("dg_stub", **kwargs)

        self.declare_parameter("publish_rate_hz", 1.0)
        rate = float(self.get_parameter("publish_rate_hz").value)

        # 로봇별 '최신값' 보관 (robot_id 키). 원본 메시지를 그대로 저장.
        self._ddago = {}   # robot_id -> DdagoTelemetry
        self._ddagi = {}   # robot_id -> DdagiTelemetry
        # robot_id -> Publisher. 로봇을 미리 알 수 없으므로(가짜 로봇이 실행 중에 늘어난다)
        # 처음 보는 robot_id 가 오면 그때 발행자를 만든다.
        self._pubs = {}

        # 모든 로봇이 한 토픽에 발행(익명) → 각 소스 토픽을 '한 번만' 구독하고 payload
        # msg.robot_id 로 가른다. telemetry_publisher 가 기본 QoS(RELIABLE,10) 발행 → 맞춤.
        self.create_subscription(
            DdagoTelemetry, "/ddago/telemetry", self._on_ddago, 10)
        self.create_subscription(
            DdagiTelemetry, "/ddagi/telemetry", self._on_ddagi, 10)

        period = 1.0 / rate if rate > 0.0 else 1.0
        self.create_timer(period, self._publish)

        self.get_logger().info(
            f"[TEST] DG 대역 준비: /ddago·/ddagi/telemetry → /<robot_id>/telemetry "
            f"({rate:.1f}Hz). robot_id(payload)로 로봇 구분. ※ 테스트 스탠드인")

    # 수신마다 robot_id 로 덮어씀(원본 보존).
    def _on_ddago(self, msg: DdagoTelemetry) -> None:
        if not msg.robot_id:
            # 실기라면 물리망이 알려주지만 이 대역은 payload 로만 로봇을 안다.
            self.get_logger().warn(
                "[TEST] robot_id 없는 ddago 텔레메트리 무시 — fake_telemetry 는 "
                "네임스페이스(-r __ns:=/dg_02)로 robot_id 를 준다",
                throttle_duration_sec=10.0)
            return
        self._ddago[msg.robot_id] = msg

    def _on_ddagi(self, msg: DdagiTelemetry) -> None:
        if not msg.robot_id:
            self.get_logger().warn(
                "[TEST] robot_id 없는 ddagi 텔레메트리 무시",
                throttle_duration_sec=10.0)
            return
        self._ddagi[msg.robot_id] = msg

    def _publisher_for(self, robot_id: str):
        pub = self._pubs.get(robot_id)
        if pub is None:
            pub = self.create_publisher(
                RobotTelemetry, robot_telemetry_topic(robot_id), 10)
            self._pubs[robot_id] = pub
            self.get_logger().info(
                f"[TEST] 새 로봇 감지 → 발행 시작: {robot_telemetry_topic(robot_id)}")
        return pub

    def _publish(self) -> None:
        # 둘 중 한쪽만 온 로봇도 발행한다(팔 없는 세트가 정상이므로).
        for robot_id in sorted(set(self._ddago) | set(self._ddagi)):
            msg = RobotTelemetry()
            # 중계 시각. 개별 ddago/ddagi header.stamp 는 로봇이 찍은 원본 그대로 둔다.
            msg.header.stamp = self.get_clock().now().to_msg()
            ddago = self._ddago.get(robot_id)
            ddagi = self._ddagi.get(robot_id)
            if ddago is not None:
                msg.ddagos = [ddago]
            if ddagi is not None:
                msg.ddagis = [ddagi]
            self._publisher_for(robot_id).publish(msg)

        self.get_logger().info(
            f"[TEST] 로봇별 텔레메트리 발행: {sorted(self._pubs)}",
            throttle_duration_sec=5.0)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DgStub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
