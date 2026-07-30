#!/usr/bin/env python3
"""그리퍼 파지 임계값(pick.GRIP_THRESHOLD) 튜닝 도구.

pick.py 는 그리퍼를 닫은 뒤 값이 임계값보다 크면 '파지 성공'으로 본다(토마토가
걸려 완전히 안 닫힘). 그래서 임계값은 이 사이에 있어야 한다:

    빈손 close 값  <  GRIP_THRESHOLD  <  토마토 물었을 때 close 값

임계값이 빈손 값보다 낮으면 → 안 잡았는데 '성공'(빈 채로 바구니). 너무 높으면 →
잡았는데 '실패'. 이 스크립트로 두 값을 재서 중간으로 잡는다.

빈손과 작은 토마토 값이 같아 구분이 안 되면(그리퍼가 무른 토마토를 거의 다 닫힌 채
잡음), CALIBRATE=1 로 '완전 닫힘=0' 캘리브레이션을 먼저 하면 값이 벌어질 수 있다.
(단, 손가락 물리위치가 정말 동일하면 캘리해도 안 갈림 → 그땐 카메라 검증으로 전환.)

실행 (노트북, 팔 연결):
    ~/venv/automato/bin/python ddagi_harvest/gripper_check.py            # 측정만
    CALIBRATE=1 ~/venv/automato/bin/python ddagi_harvest/gripper_check.py # 캘리 후 측정
    ARM_IP=192.168.x.x ... (기본 192.168.3.12)
"""
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from ddagi_harvest.arm_backend import NetworkArm  # noqa: E402


def measure_close_value(arm, label: str) -> int:
    """그리퍼를 열었다가 닫고, 닫힌 뒤의 값을 읽어 반환."""
    arm.open_gripper(100)
    time.sleep(1.2)
    input(f"[{label}] 준비되면 Enter — 그리퍼를 닫습니다")
    arm.close_gripper(100)
    time.sleep(1.5)               # 원격이라 닫힘 완료까지 고정 대기
    value = arm.gripper_value()
    print(f"  → {label} close 값 = {value}")
    return value


def calibrate(arm) -> None:
    """완전 닫힘(빈손)을 0으로 재설정 — 파지값이 빈손과 안 갈릴 때."""
    print("[캘리] 완전 닫힘을 0으로 재설정한다.")
    arm.open_gripper(100)
    time.sleep(1.2)
    input("  그리퍼를 비우고 Enter (닫은 뒤 그 위치를 0으로 잡습니다)")
    arm.close_gripper(100)
    time.sleep(1.5)
    print(f"  캘리 전 닫힘 값 = {arm.gripper_value()}")
    arm.set_gripper_calibration()
    time.sleep(1.0)
    print(f"  캘리 후 닫힘 값 = {arm.gripper_value()}  (0 근처면 성공)\n")


def main() -> int:
    ip = os.environ.get("ARM_IP", "192.168.3.12")
    do_cal = os.environ.get("CALIBRATE", "") not in ("", "0", "false")
    arm = NetworkArm(ip)
    try:
        if do_cal:
            calibrate(arm)
        print("그리퍼 임계값 측정 — 빈손과 토마토 파지 시 close 값을 잰다.\n")
        empty = measure_close_value(arm, "빈손(그리퍼에 아무것도 없이)")
        held = measure_close_value(arm, "파지(그리퍼 손끝에 토마토를 끼우고)")
        arm.open_gripper(100)

        print(f"\n빈손 = {empty}   파지 = {held}")
        if held > empty + 5:
            rec = round((empty + held) / 2)
            print(f"→ 권장 GRIP_THRESHOLD = {rec}  (두 값의 중간)")
            print(f"  pick.py 의 GRIP_THRESHOLD 를 {rec} 로 바꾸면 오탐/누락이 준다.")
        else:
            print("⚠ 파지값이 빈손보다 충분히 크지 않음 — 토마토를 제대로 못 물었거나")
            print("  값 노이즈. 토마토를 더 확실히 끼우고 다시 재보자.")
    finally:
        arm.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
