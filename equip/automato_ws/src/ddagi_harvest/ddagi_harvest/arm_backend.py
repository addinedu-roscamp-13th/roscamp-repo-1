#!/usr/bin/env python3
"""로봇팔 하드웨어 추상화 — 팔 없이 로직을 개발/검증하기 위한 계층.

파지·루프 코드는 이 인터페이스(ArmBackend)만 호출한다. 실물이 없을 땐 FakeArm
(명령 로깅 + 가짜 상태)으로 전 로직을 헤드리스로 돌리고, 팔이 비면 RealArm으로
스왑만 한다. 경쟁 구현 마감이 촉박한데 팔 1대를 3명이 나눠 쓰므로, "팔 없이
90%를 만들어두고 실물은 튜닝에만" 쓰기 위한 핵심 장치.

좌표 단위: mm / deg (pymycobot get_coords·send_coords 관례).
이동은 전부 '동기'(도달까지 블로킹)로 노출한다 — 비동기 send_coords를 고정 sleep으로
가정하다 값을 오독한 경험(학습 M1-2)을 반영.
"""
from __future__ import annotations

import time
from abc import ABC, abstractmethod


class ArmBackend(ABC):
    """파지·루프가 쓰는 최소 인터페이스."""

    @abstractmethod
    def move_coords(self, coords, speed: int = 30, mode: int = 1) -> None:
        """[x,y,z,rx,ry,rz]로 동기 이동. mode=1 직선, 0 관절보간."""

    @abstractmethod
    def move_angles(self, angles, speed: int = 30) -> None:
        """[j1..j6] 관절각으로 동기 이동 (관측/홈 자세 복귀용)."""

    @abstractmethod
    def get_coords(self):
        """현재 [x,y,z,rx,ry,rz] (base 기준)."""

    @abstractmethod
    def get_angles(self):
        """현재 [j1..j6]."""

    @abstractmethod
    def gripper(self, value: int, speed: int = 50) -> None:
        """그리퍼 값 0(닫힘)~100(열림)으로 이동 완료까지."""

    @abstractmethod
    def gripper_value(self) -> int:
        """현재 그리퍼 값."""

    # 편의 래퍼
    def open_gripper(self, speed: int = 50) -> None:
        self.gripper(100, speed)

    def close_gripper(self, speed: int = 50) -> None:
        self.gripper(0, speed)

    def grasped(self, threshold: int = 10) -> bool:
        """닫은 뒤 물체가 걸려 완전히 안 닫혔으면 파지 성공으로 본다."""
        return self.gripper_value() > threshold


class FakeArm(ArmBackend):
    """실물 없이 명령을 로깅하고 가짜 상태를 유지한다.

    - move_coords/angles: 목표를 '즉시 도달'했다고 가정하고 상태에 반영 + 로깅
    - gripper: 마지막 값 저장. grasp 성공 여부는 grip_result로 주입해 시나리오 테스트
    """

    def __init__(self, start_coords=None, start_angles=None,
                 grip_result: int | None = None, verbose: bool = True):
        self._coords = list(start_coords or [0, 0, 300, 0, 0, 0])
        self._angles = list(start_angles or [0, 0, 0, 0, 0, 0])
        self._grip = 100
        self._grip_result = grip_result  # 닫을 때 반환할 값(파지 시뮬). None=빈그리퍼(0)
        self.verbose = verbose
        self.log: list[tuple] = []

    def _rec(self, *entry):
        self.log.append(entry)
        if self.verbose:
            print("  [FakeArm]", " ".join(str(e) for e in entry))

    def move_coords(self, coords, speed=30, mode=1):
        self._coords = list(coords)
        self._rec("move_coords", [round(c, 1) for c in coords], f"spd={speed}", f"mode={mode}")

    def move_angles(self, angles, speed=30):
        self._angles = list(angles)
        self._rec("move_angles", [round(a, 1) for a in angles], f"spd={speed}")

    def get_coords(self):
        return list(self._coords)

    def get_angles(self):
        return list(self._angles)

    def gripper(self, value, speed=50):
        # 닫기(작은 값)면 파지 시뮬 값으로, 열기면 그대로.
        if value <= 10 and self._grip_result is not None:
            self._grip = self._grip_result
        else:
            self._grip = value
        self._rec("gripper", f"cmd={value}", f"-> {self._grip}")

    def gripper_value(self):
        return self._grip


class RealArm(ArmBackend):
    """pymycobot 실물. 팔이 비었을 때만 사용.

    시리얼 직결(RealArm()) 또는 소켓(RealArm(ip='192.168.x.x'))을 지원한다.
    이동은 sync_* 로 도달까지 블로킹.
    """

    def __init__(self, port="/dev/ttyUSB0", baud=1000000, ip=None, netport=9000,
                 move_timeout=15):
        from pymycobot import MyCobot280
        if ip:
            from pymycobot import MyCobot280Socket
            self.mc = MyCobot280Socket(ip, netport)
        else:
            self.mc = MyCobot280(port, baud)
        self.move_timeout = move_timeout
        time.sleep(0.5)  # 시리얼 안정화

    def move_coords(self, coords, speed=30, mode=1):
        self.mc.sync_send_coords(list(coords), speed, mode, timeout=self.move_timeout)

    def move_angles(self, angles, speed=30):
        self.mc.sync_send_angles(list(angles), speed, timeout=self.move_timeout)

    def get_coords(self):
        return self.mc.get_coords()

    def get_angles(self):
        return self.mc.get_angles()

    def gripper(self, value, speed=50):
        self.mc.set_gripper_value(value, speed)
        deadline = time.time() + 3.0
        while time.time() < deadline:
            try:
                if self.mc.is_gripper_moving() == 0:
                    break
            except Exception:
                break
            time.sleep(0.1)

    def gripper_value(self):
        return self.mc.get_gripper_value()


def make_arm(backend: str = "fake", **kwargs) -> ArmBackend:
    """'fake' 또는 'real'. 파지·루프 코드는 이걸로만 팔을 얻는다."""
    if backend == "real":
        return RealArm(**kwargs)
    return FakeArm(**kwargs)


if __name__ == "__main__":
    # FakeArm 스모크 테스트 (하드웨어 불필요)
    arm = make_arm("fake", grip_result=18)  # 18 = 토마토 걸림 시뮬
    print("=== FakeArm 스모크 ===")
    arm.move_angles([0, 0, 0, 0, 0, 0])
    arm.open_gripper()
    arm.move_coords([150, -40, 300, -90, 0, -45])
    arm.close_gripper()
    print("grasped?", arm.grasped(), "(value", arm.gripper_value(), ")")
    print("coords:", arm.get_coords())
    print(f"기록된 명령 {len(arm.log)}개  →  PASS" if arm.log else "FAIL")
