#!/usr/bin/env python3
"""RP-78 테스트 스탠드인 ① — Fleet 텔레메트리 취합 (HQ 취합 대역의 최소 흉내).

⚠️ 실제 DG Control Service(dcs) 가 아니다. 로봇별 텔레메트리를 FleetTelemetry 로 묶어
   발행하는 최소 기능만 흉내내는 '테스트 전용' 노드다. 실기 dcs 는 로봇을 물리망으로
   구분해 robot_id 를 스스로 채우지만, 이 테스트 노드는 fake_telemetry 가 payload 에
   실어주는 msg.robot_id 로 로봇을 가른다(한 머신 테스트라 물리 분리가 없으므로).

하는 일:
  구독:  /ddago/telemetry  (DdagoTelemetry)   ← 모든 로봇이 '한 토픽'에 발행, robot_id 로 구분
         /ddagi/telemetry  (DdagiTelemetry)   ← 팔 있는 로봇만(없으면 비어도 정상)
  발행:  /automato/telemetry/fleet    (FleetTelemetry, 기본 1Hz)
         ddagos[] = 로봇별 최신 DdagoTelemetry, ddagis[] = 로봇별 최신 DdagiTelemetry

핵심: 받은 DdagoTelemetry '원본'을 그대로 담는다(특히 header.stamp 보존).
  ACS의 가용 판정 staleness 는 ddago header.stamp(로봇이 찍은 시각) 기준이라,
  취합 노드가 stamp 를 새로 찍으면 안 된다. 원본을 통과시켜야 정확하다.

실행 (🖥️ 관제 PC):
  # ACS 패키지가 colcon 빌드/소싱돼 있으면
  ros2 run automato_control_service fleet_aggregator
  # 또는 모듈로 직접 (services/automato_control_service 에서, ROS 소싱 후)
  python3 -m automato_control_service.test_harness.fleet_aggregator
"""
from automato_interfaces.msg import DdagiTelemetry, DdagoTelemetry, FleetTelemetry
import rclpy
from rclpy.node import Node


class FleetAggregator(Node):
    def __init__(self, **kwargs):
        super().__init__("fleet_aggregator", **kwargs)

        self.declare_parameter("publish_rate_hz", 1.0)
        self.declare_parameter("output_topic", "/automato/telemetry/fleet")
        rate = float(self.get_parameter("publish_rate_hz").value)
        out_topic = self.get_parameter("output_topic").value

        # 로봇별 '최신값' 보관 (robot_id 키). 원본 메시지를 그대로 저장.
        self._ddago = {}   # robot_id -> DdagoTelemetry
        self._ddagi = {}   # robot_id -> DdagiTelemetry

        # 모든 로봇이 한 토픽에 발행(익명) → 각 소스 토픽을 '한 번만' 구독하고 payload
        # msg.robot_id 로 가른다. telemetry_publisher 가 기본 QoS(RELIABLE,10) 발행 → 맞춤.
        self.create_subscription(
            DdagoTelemetry, "/ddago/telemetry", self._on_ddago, 10)
        self.create_subscription(
            DdagiTelemetry, "/ddagi/telemetry", self._on_ddagi, 10)

        self._pub = self.create_publisher(FleetTelemetry, out_topic, 10)
        period = 1.0 / rate if rate > 0.0 else 1.0
        self.create_timer(period, self._publish)

        self.get_logger().info(
            f"[TEST] Fleet 취합 준비: /ddago·/ddagi/telemetry → {out_topic} "
            f"({rate:.1f}Hz). robot_id(payload)로 로봇 구분. ※ 테스트 스탠드인")

    # 수신마다 robot_id 로 덮어씀(원본 보존).
    def _on_ddago(self, msg: DdagoTelemetry) -> None:
        self._ddago[msg.robot_id] = msg

    def _on_ddagi(self, msg: DdagiTelemetry) -> None:
        self._ddagi[msg.robot_id] = msg

    def _publish(self) -> None:
        msg = FleetTelemetry()
        # 취합 메시지 자체의 시각(전체 프레임 시각). 개별 ddago header.stamp 는 원본 유지.
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.ddagos = list(self._ddago.values())
        msg.ddagis = list(self._ddagi.values())
        self._pub.publish(msg)
        self.get_logger().info(
            f"[TEST] FleetTelemetry 발행: ddago {len(msg.ddagos)}대 / "
            f"ddagi {len(msg.ddagis)}대",
            throttle_duration_sec=5.0)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FleetAggregator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
