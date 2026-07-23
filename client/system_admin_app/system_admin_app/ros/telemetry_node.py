"""ROS2 어댑터 노드.

★ 이 파일이 automato_interfaces(ROS 타입)에 의존하는 유일한 지점이다.
   FleetTelemetry 메시지를 내부 dataclass(FleetSnapshot)로 변환해서 콜백으로 넘긴다.
   msg 필드명이 팀 협의로 바뀌면 여기 _to_snapshot()만 수정하면 된다.

RP-114 대응: FleetTelemetry가 '이름은 그대로, 내용물만' 재정의되었다.
로봇별 물리망 분리로 로봇 쪽 메시지에서 robot_id가 빠지고, ACS가 취합하면서
FleetMember.robot_id로 실어 보낸다. 자세한 배경은 _to_snapshot() 주석 참조.
"""
from __future__ import annotations

from typing import Callable

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from automato_interfaces.msg import FleetTelemetry

from .. import config
from ..model.state import (
    DdagoState, DdagiState, ServoState, DGUnit, FleetSnapshot,
)


def _sensor_qos() -> QoSProfile:
    # 1Hz 스트리밍 텔레메트리: 최신값 위주, 살짝 유실 허용.
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
    )


class TelemetryNode(Node):
    def __init__(self, on_snapshot: Callable[[FleetSnapshot], None]):
        super().__init__("system_admin_app")
        self._on_snapshot = on_snapshot
        self._warned_no_robots = False   # robots 비어있음 경고는 1회만

        self.create_subscription(
            FleetTelemetry,
            config.TOPIC_FLEET_TELEMETRY,
            self._on_fleet_msg,
            _sensor_qos(),
        )

        # 제어탭 유지보수 명령 클라이언트 (QT -> ACS).
        # RobotMaintenanceCommand는 아직 팀 협의 전 '제안'이라 automato_interfaces에
        # 없을 수 있다. 없으면 제어 기능만 비활성화하고 모니터링은 정상 동작한다.
        self._maint_type = None
        self._maint_cli = None
        try:
            from automato_interfaces.srv import RobotMaintenanceCommand
            self._maint_type = RobotMaintenanceCommand
            self._maint_cli = self.create_client(
                RobotMaintenanceCommand, config.SERVICE_MAINTENANCE
            )
        except (ImportError, ModuleNotFoundError):
            self.get_logger().warn(
                "RobotMaintenanceCommand 인터페이스 없음 — 제어탭 명령 비활성 "
                "(인터페이스 확정 후 활성화)"
            )

        self.get_logger().info(
            f"구독 시작: {config.TOPIC_FLEET_TELEMETRY}"
        )

    @property
    def maintenance_available(self) -> bool:
        return self._maint_cli is not None

    # ---- 구독 콜백 (ROS 실행 스레드에서 호출됨) ----
    def _on_fleet_msg(self, msg: FleetTelemetry) -> None:
        try:
            snap = self._to_snapshot(msg)
        except Exception as exc:  # 파싱 실패가 앱을 죽이지 않도록
            self.get_logger().error(f"FleetTelemetry 파싱 실패: {exc}")
            return
        self._on_snapshot(snap)

    def _to_snapshot(self, msg: FleetTelemetry) -> FleetSnapshot:
        """FleetTelemetry(재정의판) → 내부 스냅샷.

        RP-114로 구조가 바뀌었다. 토픽·타입 이름은 그대로지만 내용물이 다르다.
          이전: msg.ddagos[] / msg.ddagis[] 를 각 원소의 robot_id로 묶었다.
          이후: msg.robots[] (FleetMember) — robot_id는 여기에만 있고,
                실제 데이터는 member.telemetry(RobotTelemetry) 안에 있다.

        로봇 쪽 메시지의 robot_id 필드는 '[삭제 예정]'으로 남아 있으나 **빈 문자열**이라
        그걸로 묶으면 세 로봇이 "" 하나로 뭉개진다. 반드시 member.robot_id를 쓴다.
        """
        now_sec = self.get_clock().now().nanoseconds / 1e9
        units: dict[str, DGUnit] = {}

        if not msg.robots and not self._warned_no_robots:
            self._warned_no_robots = True
            self.get_logger().warn(
                "FleetTelemetry.robots가 비어 있다 — 발행측이 아직 재정의 이전 구조로 "
                "보내는지 확인 필요 (ddagos/ddagis는 삭제 예정 필드라 읽지 않는다)"
            )

        for member in msg.robots:
            robot_id = member.robot_id
            tel = member.telemetry

            # 이 로봇 데이터가 얼마나 오래된 값인지. ACS는 끊긴 로봇도 마지막 값을
            # 계속 실어 보내므로, 이 나이가 유일한 생존 판정 근거다.
            stamp = tel.header.stamp
            age_sec = now_sec - (stamp.sec + stamp.nanosec / 1e9)

            unit = DGUnit(robot_id=robot_id, age_sec=age_sec)

            # ddagos/ddagis는 세트당 0개 또는 1개(옵셔널 표현이 배열뿐이라 길이 0/1).
            for d in tel.ddagos:
                unit.ddago = DdagoState(
                    robot_id=robot_id,
                    task_id=int(d.task_id),
                    nav_status=d.nav_status,
                    is_charging=bool(d.is_charging),
                    x=float(d.x), y=float(d.y), yaw=float(d.yaw),
                    battery_percent=float(d.battery_percent),
                    battery_voltage=float(d.battery_voltage),
                )

            for a in tel.ddagis:
                servos = [
                    ServoState(
                        joint_no=int(s.joint_no),
                        voltage_ok=bool(s.voltage_ok),
                        temperature=int(s.temperature),
                        current=float(s.current),
                        overload=bool(s.overload),
                        gripper_value=int(s.gripper_value),
                    )
                    for s in a.servo_health
                ]
                unit.ddagi = DdagiState(
                    robot_id=robot_id,
                    task_id=int(a.task_id),
                    is_paused=bool(a.is_paused),
                    joint_angles=list(a.joint_angles),
                    tcp_coords=list(a.tcp_coords),
                    servos=servos,
                )

            units[robot_id] = unit

        return FleetSnapshot(units=units)

    # ---- 제어탭에서 호출하는 유지보수 명령 (비동기) ----
    def send_maintenance(
        self, robot_id: str, command: str,
        linear_x: float = 0.0, angular_z: float = 0.0,
    ):
        """인터페이스/서비스가 없으면 즉시 None 반환(호출측에서 처리)."""
        if self._maint_cli is None:
            return None
        if not self._maint_cli.service_is_ready():
            # 논블로킹으로 한 번 대기 시도
            self._maint_cli.wait_for_service(timeout_sec=0.2)
        if not self._maint_cli.service_is_ready():
            return None
        req = self._maint_type.Request()
        req.robot_id = robot_id
        req.command = command
        req.linear_x = float(linear_x)
        req.angular_z = float(angular_z)
        return self._maint_cli.call_async(req)
