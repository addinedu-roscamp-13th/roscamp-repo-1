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


class NetworkArm(ArmBackend):
    """노트북에서 Pi의 arm_server로 팔을 원격 조종 (개발 편의용).

    카메라가 노트북에 있어 전 파이프라인을 노트북에서 돌리려는 개발 단계용.
    최종 배포는 Pi 로컬 RealArm. 인터페이스가 같아 pick/루프 코드는 그대로.
    """

    def __init__(self, ip, port=9010, move_timeout=15, connect_timeout=5):
        import socket
        self._sock = socket.create_connection((ip, port), timeout=connect_timeout)
        self._sock.settimeout(move_timeout + 10)
        self._f = self._sock.makefile("rwb")
        self.move_timeout = move_timeout

    def _call(self, method, *args):
        import json
        self._f.write((json.dumps({"m": method, "a": list(args)}) + "\n").encode())
        self._f.flush()
        resp = json.loads(self._f.readline().decode())
        if not resp.get("ok"):
            raise RuntimeError(f"arm_server 오류({method}): {resp.get('err')}")
        return resp.get("r")

    # ---- 이동 완료 판정 -------------------------------------------------- #
    # pymycobot 의 sync_send_* 는 is_in_position(펌웨어 허용오차)이 1을 돌려줄 때까지
    # 폴링한다. 우리 목표엔 늘 수 mm 잔차가 있어 그게 안 맞는 일이 흔하고, 그러면
    # **timeout 전체(15s)를 낭비**한다(팔은 이미 2초에 멈췄는데). 도달 불가면 아예 안
    # 움직이는데도 15초를 기다린다. 그래서 async send_* + '실제 정지 감지'로 바꿨다.
    poll_dt = 0.12            # 위치 폴링 간격(s)
    stall_polls = 4           # 이만큼 연속 '안 움직임'이면 정지로 판정
    move_start_timeout = 1.2  # 이 시간 안에 안 움직이면 도달 불가로 보고 즉시 종료
    post_move_settle = 0.25   # 정지 감지 후 정착 대기 — 서보가 목표로 마지막 수 mm를
                              # 좁히는 구간을 잘라먹으면 그리퍼가 허공에서 닫힌다

    def _wait_until_stopped(self, kind: str, tol: float, timeout: float):
        """이동 명령 후 정지까지 대기. (움직였나, 마지막 위치) 반환.

        kind: 'coords'|'angles'. tol: 폴링 간 '움직임'으로 볼 최소 변화(mm 또는 deg).
        """
        getter = "get_coords" if kind == "coords" else "get_angles"
        prev = self._call(getter) or []
        started, stable, t0 = False, 0, time.time()
        while time.time() - t0 < timeout:
            time.sleep(self.poll_dt)
            cur = self._call(getter)
            if not cur:
                continue
            delta = (sum(abs(cur[i] - prev[i]) for i in range(3)) if prev else 0.0)
            prev = cur
            if delta > tol:
                started, stable = True, 0
            elif started:
                stable += 1
                if stable >= self.stall_polls:
                    break                       # 움직였다가 멈춤 = 도착
            elif time.time() - t0 > self.move_start_timeout:
                break                           # 아예 안 움직임 = 도달 불가(즉시 종료)
        if started:
            time.sleep(self.post_move_settle)   # 마지막 수 mm 정착 대기
            prev = self._call(getter) or prev
        return started, prev

    def move_coords(self, coords, speed=30, mode=1):
        # myCobot은 도달 불가(IK 해 없음) 좌표를 조용히 무시한다. '안 움직였고 목표와
        # 멀면' 거부로 본다 — 정지 감지 덕에 15초 대기 없이 ~1초에 판정된다.
        before = self._call("get_coords") or [0, 0, 0, 0, 0, 0]
        self._call("send_coords", list(coords), speed, mode)
        started, after = self._wait_until_stopped("coords", 1.5, self.move_timeout)
        after = after or before
        tgt = list(coords)[:3]
        moved = sum(abs(after[i] - before[i]) for i in range(3))
        err = sum(abs(after[i] - tgt[i]) for i in range(3))
        if (not started or moved < 5) and err > 40:
            raise RuntimeError(
                f"도달 불가(IK 해 없음): 목표 {[round(t, 1) for t in tgt]} "
                f"— 작업공간 밖이거나 그 자세로 그 위치 불가")

    def move_angles(self, angles, speed=30):
        self._call("send_angles", list(angles), speed)
        self._wait_until_stopped("angles", 0.5, self.move_timeout)

    def get_coords(self):
        return self._call("get_coords")

    def get_angles(self):
        return self._call("get_angles")

    # 원격이라 is_gripper_moving 폴링(왕복 여러 번) 대신 고정 대기. 픽당 그리퍼 명령이
    # 3회(열기·파지·상승후확인)라 이 값이 사이클에 ×3으로 누적된다.
    gripper_wait = 0.6

    def gripper(self, value, speed=50):
        self._call("set_gripper_value", value, speed)
        time.sleep(self.gripper_wait)

    def gripper_value(self):
        return self._call("get_gripper_value")

    def set_gripper_calibration(self):
        """현재 그리퍼 위치를 닫힘(0) 기준으로 캘리브레이션(파지값 분해능 개선용)."""
        self._call("set_gripper_calibration")

    # 드래그 티칭 지원 (관측자세 잡을 때)
    def release_servos(self):
        self._call("release_all_servos")

    def focus_servos(self):
        self._call("focus_all_servos")

    def close(self):
        try:
            self._f.close()
            self._sock.close()
        except Exception:
            pass


def make_arm(backend: str = "fake", **kwargs) -> ArmBackend:
    """'fake' / 'real' / 'network'. 파지·루프 코드는 이걸로만 팔을 얻는다."""
    if backend == "real":
        return RealArm(**kwargs)
    if backend == "network":
        return NetworkArm(**kwargs)
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
