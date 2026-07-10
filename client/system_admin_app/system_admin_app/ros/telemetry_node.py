"""ROS2 어댑터 노드.

★ 이 파일이 automato_interfaces(ROS 타입)에 의존하는 유일한 지점이다.
   FleetTelemetry 메시지를 내부 dataclass(FleetSnapshot)로 변환해서 콜백으로 넘긴다.
   msg 필드명이 팀 협의로 바뀌면 여기 _to_snapshot()만 수정하면 된다.
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

    @staticmethod
    def _to_snapshot(msg: FleetTelemetry) -> FleetSnapshot:
        units: dict[str, DGUnit] = {}

        def unit_of(robot_id: str) -> DGUnit:
            u = units.get(robot_id)
            if u is None:
                u = DGUnit(robot_id=robot_id)
                units[robot_id] = u
            return u

        for d in msg.ddagos:
            unit_of(d.robot_id).ddago = DdagoState(
                robot_id=d.robot_id,
                task_id=int(d.task_id),
                nav_status=d.nav_status,
                is_charging=bool(d.is_charging),
                x=float(d.x), y=float(d.y), yaw=float(d.yaw),
                battery_percent=float(d.battery_percent),
                battery_voltage=float(d.battery_voltage),
                us_range_m=float(d.us_range_m),
            )

        for a in msg.ddagis:
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
            unit_of(a.robot_id).ddagi = DdagiState(
                robot_id=a.robot_id,
                task_id=int(a.task_id),
                is_paused=bool(a.is_paused),
                joint_angles=list(a.joint_angles),
                tcp_coords=list(a.tcp_coords),
                servos=servos,
            )

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
