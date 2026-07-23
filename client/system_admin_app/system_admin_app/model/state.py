"""내부 상태 모델 (ROS 메시지와 분리된 순수 파이썬 dataclass).

ROS 타입(automato_interfaces)에 대한 의존은 ros/telemetry_node.py 어댑터에만 두고,
UI/모델은 이 dataclass들만 사용한다. 나중에 msg 필드명이 바뀌어도 어댑터 한 곳만
고치면 되도록 하기 위함.
"""
from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Optional

from .. import config


@dataclass
class ServoState:
    joint_no: int
    voltage_ok: bool
    temperature: int
    current: float
    overload: bool
    gripper_value: int  # 7번(그리퍼)에서만 의미. 0~100(%)로 표시.


@dataclass
class DdagoState:
    """주행 로봇 텔레메트리."""
    robot_id: str
    task_id: int = 0
    nav_status: str = ""
    is_charging: bool = False        # 미사용 확정 필드 (E0 note). UI에서 표시 안 함.
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    battery_percent: float = 0.0
    battery_voltage: float = 0.0
    # 초음파(us_range)는 실제 데이터가 오지 않아 모니터링에서 제외 — 필드도 두지 않는다.
    rx_time: float = field(default_factory=time.monotonic)  # 수신 시각(monotonic)


@dataclass
class DdagiState:
    """로봇팔 텔레메트리."""
    robot_id: str
    task_id: int = 0
    is_paused: bool = False
    joint_angles: list[float] = field(default_factory=list)   # 6
    tcp_coords: list[float] = field(default_factory=list)     # 6 (x,y,z,rx,ry,rz)
    servos: list[ServoState] = field(default_factory=list)    # 7
    rx_time: float = field(default_factory=time.monotonic)

    @property
    def arm_servos(self) -> list[ServoState]:
        """그리퍼(7번)를 제외한 6관절 서보."""
        return [s for s in self.servos if s.joint_no != config.GRIPPER_JOINT_NO]

    @property
    def gripper(self) -> Optional[ServoState]:
        for s in self.servos:
            if s.joint_no == config.GRIPPER_JOINT_NO:
                return s
        return None

    @property
    def max_temperature(self) -> Optional[int]:
        temps = [s.temperature for s in self.servos]
        return max(temps) if temps else None

    @property
    def any_overload(self) -> bool:
        return any(s.overload for s in self.servos)

    @property
    def any_undervoltage(self) -> bool:
        return any(not s.voltage_ok for s in self.servos)

    @property
    def gripper_percent(self) -> Optional[int]:
        g = self.gripper
        return g.gripper_value if g else None


@dataclass
class DGUnit:
    """dg_0N 단위. 주행(Ddago)과 로봇팔(Ddagi)을 하나로 묶되 각각 접근 가능."""
    robot_id: str
    ddago: Optional[DdagoState] = None
    ddagi: Optional[DdagiState] = None
    # 이 로봇 텔레메트리의 나이(초). ACS가 실어 보낸 header.stamp와 수신 시각의 차.
    age_sec: Optional[float] = None

    @property
    def is_drive_only(self) -> bool:
        return self.robot_id in config.DRIVE_ONLY_ROBOTS

    @property
    def is_offline(self) -> bool:
        """통신 두절 여부 (E0 ④ 규격: 3초 이상 미갱신).

        ACS는 텔레메트리가 끊긴 로봇도 배열에서 빼지 않고 마지막 값을 계속 보낸다
        (빼면 화면에서 로봇이 깜빡이고 '연결 끊김'과 '존재하지 않음'을 구분할 수 없다).
        따라서 **배열에 있다는 것이 살아있다는 뜻이 아니며**, stamp의 나이로만 판정한다.
        """
        return self.age_sec is not None and self.age_sec >= config.STALE_SEC


@dataclass
class FleetSnapshot:
    """한 프레임의 편대 전체 상태. ROS 스레드 → GUI 스레드로 시그널에 실려 넘어간다."""
    units: dict[str, DGUnit] = field(default_factory=dict)
    stamp: float = field(default_factory=time.monotonic)

    def unit(self, robot_id: str) -> Optional[DGUnit]:
        return self.units.get(robot_id)
