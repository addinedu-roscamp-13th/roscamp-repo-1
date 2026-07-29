#!/usr/bin/env python3
"""그리퍼 캘리 복구 — set_gripper_calibration 을 '닫힘'에서 잘못 불러 스케일이
뒤집혔을 때(닫힘이 100으로 읽힘) 되돌린다.

올바른 캘리: 그리퍼를 **완전히 연 상태**에서 set_gripper_calibration() 호출 → 그
위치가 열림(100) 기준이 되고 닫힘이 0이 된다. 이 스크립트는 그리퍼를 열고(어느
명령이 여는지 관찰로 확인) 그 상태에서 재캘리한 뒤 닫힘0/열림100 을 검증한다.

⚠ 그리퍼를 계속 지켜봐. 이상하게 움직이면 Ctrl+C.
실행: ~/venv/automato/bin/python ddagi_harvest/gripper_recover.py
"""
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from ddagi_harvest.arm_backend import NetworkArm  # noqa: E402


def _cmd(arm, value, speed=40):
    """set_gripper_value 직접 호출(스케일이 뒤집혔을 수 있어 open/close 래퍼 안 씀)."""
    arm._call("set_gripper_value", value, speed)
    time.sleep(2.0)
    return arm.gripper_value()


def main() -> int:
    ip = os.environ.get("ARM_IP", "192.168.100.12")
    arm = NetworkArm(ip)
    try:
        print(f"현재 그리퍼 값 = {arm.gripper_value()}\n")

        print("그리퍼를 완전히 열어야 한다. 어느 명령이 여는지 관찰로 찾자.")
        v100 = _cmd(arm, 100)
        print(f"  set_gripper_value(100) → 값 {v100}")
        if input("  그리퍼가 '완전히 열렸'나? (y/n): ").strip().lower() != "y":
            v0 = _cmd(arm, 0)
            print(f"  set_gripper_value(0) → 값 {v0}")
            if input("  이제 완전히 열렸나? (y/n): ").strip().lower() != "y":
                input("  둘 다 아니면 손으로 완전 열림 위치로 두고 Enter")

        input("\n완전 열림 확인됐으면 Enter — 이 위치를 기준으로 재캘리한다")
        arm.set_gripper_calibration()
        time.sleep(1.0)
        print(f"  재캘리 완료. 현재 값 = {arm.gripper_value()}")

        print("\n검증:")
        c = _cmd(arm, 0)
        print(f"  닫기 → 값 {c}  (0 근처여야 정상)")
        o = _cmd(arm, 100)
        print(f"  열기 → 값 {o}  (100 근처여야 정상)")
        if c < 20 and o > 80:
            print("\n✓ 복구 성공 — 닫힘0/열림100 정상. 이제 캘리는 건드리지 말자.")
        else:
            print("\n⚠ 아직 이상 — 이 값들 알려줘. 다른 방법 찾자.")
    finally:
        arm.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
