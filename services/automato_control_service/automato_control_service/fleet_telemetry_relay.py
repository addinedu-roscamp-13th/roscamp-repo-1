#!/usr/bin/env python3
"""RP-77  E0 ③④: Fleet 텔레메트리 취합·QT 대시보드 중계.

HQ(DG Control Service)가 로봇 전체의 텔레메트리를 하나로 취합해 발행하는
FleetTelemetry(/automato/telemetry/fleet)를 받아, System Admin App(QT)용
토픽(/automato/dashboard/fleet_telemetry)으로 그대로 재발행한다.

QT는 시스템 진단이 목적이라 축약·가공 없이 로봇 원본 데이터를 손실 없이 통과시킨다.
저장(DB/메모리/rosbag2) 없음 — 실시간 릴레이 전용.

방식: 직접 릴레이(relay). 구독 콜백에서 받은 메시지를 즉시 재발행한다.
입력이 1Hz라 출력도 자연히 1Hz. HQ가 멈추면 출력도 멈춰, QT가 연결 끊김을
그대로 관측할 수 있다(진단 목적에 부합 — stale 데이터를 지어내지 않음).

  구독:  /automato/telemetry/fleet          automato_interfaces/FleetTelemetry
  발행:  /automato/dashboard/fleet_telemetry automato_interfaces/FleetTelemetry (동일 타입 재사용)

파라미터:
  input_topic   (str)  구독할 HQ 취합 토픽      기본 /automato/telemetry/fleet
  output_topic  (str)  QT 대시보드용 재발행 토픽 기본 /automato/dashboard/fleet_telemetry
"""
from automato_interfaces.msg import FleetTelemetry
import rclpy
from rclpy.node import Node


class FleetTelemetryRelay(Node):
    def __init__(self, **kwargs):
        # **kwargs 는 테스트에서 parameter_overrides 등을 주입하기 위한 통로.
        super().__init__('fleet_telemetry_relay', **kwargs)

        # --- 파라미터 (기본값이 시퀀스 다이어그램 스펙 그대로) ---
        self.declare_parameter('input_topic', '/automato/telemetry/fleet')
        self.declare_parameter(
            'output_topic', '/automato/dashboard/fleet_telemetry')
        self._in_topic = self.get_parameter('input_topic').value
        out_topic = self.get_parameter('output_topic').value

        # --- 발행 + 구독 ---
        # 진단용 1Hz 텔레메트리라 기본 QoS(RELIABLE, depth 10)로 맞춘다
        # (RP-75 ddago/telemetry 발행 관례와 동일 — HQ 발행자도 RELIABLE 로 매칭).
        self._pub = self.create_publisher(FleetTelemetry, out_topic, 10)
        self._sub = self.create_subscription(
            FleetTelemetry, self._in_topic, self._relay, 10)

        # --- 가시성 보조 상태 (재발행 로직과 무관, 로그 전용) ---
        self._last_rx = None    # 마지막 수신 시각(rclpy Time)
        # 워치독: 1초마다 입력이 끊겼는지 확인해 경고만 한다(재발행 안 함).
        self._watchdog = self.create_timer(1.0, self._check_input)

        self.get_logger().info(
            f'Fleet 텔레메트리 릴레이 준비: {self._in_topic} → {out_topic} '
            '(직접 릴레이, 저장 없음)'
        )

    # ------------------------------------------------------------------ #
    # 릴레이: 받은 원본을 저장·가공 없이 그대로 통과 (배열/헤더 손실 없음)
    # ------------------------------------------------------------------ #
    def _relay(self, msg):
        self._pub.publish(msg)
        self._last_rx = self.get_clock().now()
        # 매 프레임 찍으면 도배되므로 5초에 한 번만 흐름을 보여준다.
        self.get_logger().info(
            f'릴레이: ddago {len(msg.ddagos)}대 / ddagi {len(msg.ddagis)}대',
            throttle_duration_sec=5.0,
        )

    # ------------------------------------------------------------------ #
    # 워치독: 입력이 끊기면 경고 (직접 릴레이는 입력 끊기면 조용히 멈추므로 안내)
    # ------------------------------------------------------------------ #
    def _check_input(self):
        if self._last_rx is None:
            self.get_logger().warn(
                f'{self._in_topic} 입력 대기 중 — HQ FleetTelemetry 발행 확인 필요',
                throttle_duration_sec=5.0,
            )
            return
        age = (self.get_clock().now() - self._last_rx).nanoseconds * 1e-9
        if age > 3.0:
            self.get_logger().warn(
                f'입력 {age:.1f}s 끊김 — 재발행도 멈춤(진단 목적상 정상 동작)',
                throttle_duration_sec=5.0,
            )


def main(args=None):
    rclpy.init(args=args)
    node = FleetTelemetryRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
