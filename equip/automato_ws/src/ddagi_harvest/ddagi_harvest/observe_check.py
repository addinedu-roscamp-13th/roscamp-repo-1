#!/usr/bin/env python3
"""관측자세 도달 점검 — 명령한 관절각과 실제 도달각의 차이를 잰다.

왜 필요한가: TF(camera→base)는 URDF 순기구학으로 base←joint6 를 구하는데, 그 입력이
**명령값 OBSERVE_ANGLES** 다. 팔이 그 각도에 실제로 도달하지 못하면(특히 카메라를
직접 돌리는 J5) 계산상의 카메라 자세와 실제가 어긋나 모든 base 좌표가 틀어진다.

두 경우를 가른다:
  ① 실측각 ≠ 명령각  → 팔이 도달을 못 하는 것. FK 입력을 '실측각'으로 바꾸면 해결.
  ② 실측각 = 명령각인데 화면은 돌아가 있음 → 엔코더 영점이 밀린 것. 재티칭 필요.

실행:
    ~/venv/automato/bin/python ddagi_harvest/observe_check.py
    ARM_IP=192.168.x.x ...  (기본 192.168.100.12)
"""
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from ddagi_harvest import pick as pk            # noqa: E402
from ddagi_harvest.arm_backend import NetworkArm  # noqa: E402

REPEATS = 3          # 여러 번 반복해 재현성(매번 같은 오차인지) 확인


def main() -> int:
    ip = os.environ.get("ARM_IP", "192.168.100.12")
    arm = NetworkArm(ip)
    cmd = pk.OBSERVE_ANGLES
    try:
        print(f"명령 OBSERVE_ANGLES = {[round(a, 1) for a in cmd]}\n")
        for n in range(1, REPEATS + 1):
            # 매번 다른 자세에서 출발해야 '도달 실패'가 드러난다
            arm.move_angles(pk.STAGING_ANGLES, 40)
            time.sleep(0.3)
            arm.move_angles(cmd, 30)
            time.sleep(1.0)                       # 충분히 정착시킨 뒤 읽기
            act = arm.get_angles()
            if not act:
                print(f"  [{n}] 각도 읽기 실패")
                continue
            d = [act[i] - cmd[i] for i in range(6)]
            print(f"  [{n}] 실측 {[round(a, 1) for a in act]}")
            print(f"      차이 {[round(x, 1) for x in d]}"
                  f"   (J5 차이 {d[4]:+.1f}°)")
        coords = arm.get_coords()
        if coords:
            print(f"\n관측자세 get_coords = {[round(c, 1) for c in coords]}")
        print("\n판정 기준:")
        print("  · J5 차이가 매번 1° 이상 같은 방향 → 도달 실패. FK 입력을 실측각으로 교체")
        print("  · 차이가 거의 0 인데 화면이 돌아가 있음 → 엔코더 영점 밀림. 재티칭 필요")
    finally:
        arm.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
