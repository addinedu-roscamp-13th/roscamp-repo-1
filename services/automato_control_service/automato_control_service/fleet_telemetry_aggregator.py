#!/usr/bin/env python3
"""RP-114  E0 ③④: Fleet 텔레메트리 취합 → QT 대시보드 발행.

RP-77 때 이 노드는 '릴레이'였다 — DG 가 로봇 3대분을 하나로 묶어 보내주면 그걸 그대로
QT 토픽에 통과시키기만 했다. 그런데 DG 는 로봇 세트마다 하나씩 뜨므로 애초에 로봇 전체를
볼 수 없었다(로봇당 하나인데 3대분 배열을 갖는 잘못된 설계). RP-114 로 취합 책임이
ACS 로 넘어오면서 이 노드는 '취합기'가 되었다.

  구독:  /{robot_id}/telemetry            automato_interfaces/RobotTelemetry  (로봇 수만큼)
  구독:  /automato/telemetry/fleet        automato_interfaces/FleetTelemetry  [삭제 예정] 옛 경로
  발행:  /automato/dashboard/fleet_telemetry  automato_interfaces/FleetTelemetry (1Hz, 취합)

발행 방식이 릴레이와 다르다 — 수신할 때 발행하지 않고 **자체 1Hz 타이머**로 발행한다.
로봇 3대의 도착 시각이 제각각이라 수신에 맞춰 발행하면 프레임이 불규칙해지고, 한 대만
빨라도 그 로봇 기준으로 화면이 갱신된다. 캐시에 최신값을 쌓아두고 정해진 박자로 내보낸다.

끊긴 로봇도 배열에서 빼지 않는다(마지막 값 유지). QT 는 각 telemetry.header.stamp 와
현재 시각을 비교해 3초 이상이면 오프라인으로 표시한다 — 판정은 QT 가 하고 우리는 사실만 전한다.

QT 는 시스템 진단이 목적이라 축약·가공 없이 로봇 원본 데이터를 손실 없이 통과시킨다.
저장(DB/메모리/rosbag2) 없음 — 실시간 스트리밍 전용.

파라미터:
  robot_ids         (str[])  구독할 로봇 목록          기본 [dg_01, dg_02, dg_03]
  output_topic      (str)    QT 대시보드용 발행 토픽    기본 /automato/dashboard/fleet_telemetry
  publish_rate_hz   (float)  취합 발행 주기            기본 1.0
  legacy_input      (bool)   옛 fleet 토픽도 함께 구독  기본 True [삭제 예정]
"""
import time

from automato_interfaces.msg import FleetTelemetry
import rclpy
from rclpy.node import Node

from automato_control_service.fleet_collector import (
    DEFAULT_ROBOT_IDS,
    LEGACY_FLEET_TOPIC,
    FleetCollector,
    robot_telemetry_topic,
    subscribe_per_robot,
)

# 이 시간(초) 넘게 텔레메트리가 안 오면 경고 로그. QT 오프라인 표시 기준과 같은 값이다.
STALE_SEC = 3.0


class FleetTelemetryAggregator(Node):
    def __init__(self, **kwargs):
        # **kwargs 는 테스트에서 parameter_overrides 등을 주입하기 위한 통로.
        super().__init__('fleet_telemetry_aggregator', **kwargs)

        # --- 파라미터 ---
        self.declare_parameter('robot_ids', DEFAULT_ROBOT_IDS)
        self.declare_parameter(
            'output_topic', '/automato/dashboard/fleet_telemetry')
        self.declare_parameter('publish_rate_hz', 1.0)
        self.declare_parameter('legacy_input', True)
        robot_ids = list(self.get_parameter('robot_ids').value)
        out_topic = self.get_parameter('output_topic').value
        rate = float(self.get_parameter('publish_rate_hz').value)
        legacy_input = bool(self.get_parameter('legacy_input').value)

        self._collector = FleetCollector()
        self._last_rx = {}              # robot_id -> 마지막 수신 시각(로그·워치독 전용)
        self._first_rx_logged = set()   # 로봇별 첫 수신을 1회만 INFO 로 알리기 위한 표시

        # --- 구독: 로봇별 /{robot_id}/telemetry ---
        # 진단용 1Hz 텔레메트리라 기본 QoS(RELIABLE, depth 10) — DG 발행자와 맞춘다.
        subscribe_per_robot(self, robot_ids, self._on_robot_telemetry)

        # --- 구독: [삭제 예정] 옛 경로 ---
        # 팀원의 DG(dg_control) 이전이 끝나기 전까지, 옛 형식으로 오는 것도 받아 같은
        # 캐시에 병합한다. 이전이 끝나면 legacy_input:=false 로 끄고 관련 코드를 지운다.
        if legacy_input:
            self.create_subscription(
                FleetTelemetry, LEGACY_FLEET_TOPIC, self._on_legacy_fleet, 10)

        # --- 발행 + 취합 타이머 ---
        self._pub = self.create_publisher(FleetTelemetry, out_topic, 10)
        period = 1.0 / rate if rate > 0.0 else 1.0
        self.create_timer(period, self._publish)

        self.get_logger().info(
            'Fleet 텔레메트리 취합 준비: %s → %s (%.1fHz, 저장 없음)%s'
            % ([robot_telemetry_topic(r) for r in robot_ids], out_topic, rate,
               ', 옛 %s 도 함께 구독' % LEGACY_FLEET_TOPIC if legacy_input else '')
        )

    # ------------------------------------------------------------------ #
    # 수신 — 캐시에 최신값만 쌓는다(발행은 타이머가 한다)
    # ------------------------------------------------------------------ #
    def _on_robot_telemetry(self, robot_id, msg):
        self._collector.update(robot_id, msg)
        self._last_rx[robot_id] = time.time()

        if robot_id not in self._first_rx_logged:   # 첫 수신은 로봇마다 1회 확실히 알림
            self._first_rx_logged.add(robot_id)
            self.get_logger().info(
                '%s 첫 수신: ddago %d / ddagi %d → 캐시 갱신'
                % (robot_id, len(msg.ddagos), len(msg.ddagis)))
        else:                                       # 이후 상세는 DEBUG (1Hz 도배 방지)
            self.get_logger().debug(
                '%s 수신: ddago %d / ddagi %d'
                % (robot_id, len(msg.ddagos), len(msg.ddagis)),
                throttle_duration_sec=5.0)

    def _on_legacy_fleet(self, msg):
        """[삭제 예정] 옛 /automato/telemetry/fleet 수신 → 로봇별로 갈라 같은 캐시에 병합."""
        skipped_ddago, skipped_ddagi = self._collector.update_from_legacy_fleet(msg)
        now = time.time()
        for robot_id in self._collector.robot_ids():
            self._last_rx[robot_id] = now

        if skipped_ddago or skipped_ddagi:
            # robot_id 가 빈 항목은 어느 로봇인지 알 수 없어 버렸다. 조용히 사라지면
            # "왜 로봇이 안 보이지"를 추적할 수 없으므로 반드시 남긴다.
            self.get_logger().warn(
                '옛 fleet 에서 robot_id 없는 항목 무시: ddago %d / ddagi %d '
                '— DG 가 payload robot_id 를 채우는지 확인 필요'
                % (skipped_ddago, skipped_ddagi),
                throttle_duration_sec=10.0)
        self.get_logger().info(
            '[삭제 예정] 옛 fleet 경로로 수신 중 — DG 이전 후 legacy_input 을 끄세요',
            throttle_duration_sec=30.0)

    # ------------------------------------------------------------------ #
    # 발행 — 자체 박자로 '지금의 최신값'을 모아 한 프레임으로
    # ------------------------------------------------------------------ #
    def _publish(self):
        msg = self._collector.build_fleet_message(
            self.get_clock().now().to_msg())
        self._pub.publish(msg)

        if not msg.robots:
            self.get_logger().warn(
                '아직 받은 로봇 텔레메트리가 없음 — 빈 배열 발행 중 '
                '(DG 발행과 robot_ids 파라미터 확인 필요)',
                throttle_duration_sec=5.0)
            return

        # 오래된 로봇을 함께 알린다. 배열에서 빼지는 않는다(QT 가 stamp 로 판정).
        now = time.time()
        stale = [r for r in self._collector.robot_ids()
                 if now - self._last_rx.get(r, 0.0) > STALE_SEC]
        self.get_logger().info(
            '취합 발행: 로봇 %d대%s'
            % (len(msg.robots),
               ' (미수신 %.0fs+: %s)' % (STALE_SEC, ', '.join(stale)) if stale else ''),
            throttle_duration_sec=5.0)


def main(args=None):
    rclpy.init(args=args)
    node = FleetTelemetryAggregator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
