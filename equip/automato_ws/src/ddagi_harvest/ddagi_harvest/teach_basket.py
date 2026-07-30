#!/usr/bin/env python3
"""바구니 자세 티칭 — pick.py 의 BASKET_* 상수를 드래그 티칭으로 다시 잡는다.

바구니를 옮기면 이 3개를 다시 잡아야 한다:
    ① 접근(공용)   : 바구니 '위'의 자세. 상품/폐기품 공용. 여기서 놓기로 내려간다.
                     낮은 파지 자세에서 곧장 바구니로 가면 벽을 치므로 이 경유가 필수.
    ② 상품 놓기    : 정상품(NORMAL) 바구니에 손을 넣어 놓는 자세.
    ③ 폐기품 놓기  : 폐기품(DISCARD) 바구니에 놓는 자세.

투하 시퀀스는 'J1만 회전 → ① 접근 → 재확인 → ②/③ 놓기 → ① 로 복귀'.
그래서 ①은 두 바구니 어느 쪽으로도 내려갈 수 있는 위치여야 한다.

실행 (노트북, 팔 연결. arm_server 가 Pi 에서 떠 있어야 함):
    ~/venv/automato/bin/python ddagi_harvest/teach_basket.py
    ARM_IP=192.168.x.x ... (기본 192.168.100.12)

!! 서보를 풀면 팔이 중력으로 처진다. 반드시 손으로 받친 뒤 진행.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from ddagi_harvest.arm_backend import NetworkArm  # noqa: E402

STEPS = [
    ("approach", "① 접근(공용) — 두 바구니 '위'의 경유 자세"),
    ("normal", "② 상품(NORMAL) 놓기 — 정상품 바구니에 놓는 자세"),
    ("discard", "③ 폐기품(DISCARD) 놓기 — 폐기품 바구니에 놓는 자세"),
]


def _fmt(vals) -> str:
    return "[" + ", ".join(f"{v:.1f}" for v in vals) + "]"


def main() -> int:
    ip = os.environ.get("ARM_IP", "192.168.100.12")
    arm = NetworkArm(ip)
    captured: dict = {}
    try:
        print(__doc__)
        if input("서보를 풀고 티칭을 시작할까요? (y/N) ").strip().lower() != "y":
            return 0

        for key, label in STEPS:
            print(f"\n{label}")
            arm.release_servos()
            input("  서보 해제됨 — 팔을 받치고 그 자세로 옮긴 뒤 Enter")
            angles = arm.get_angles()
            arm.focus_servos()
            if not angles:
                print("  각도 못 읽음 — 이 단계 건너뜀(다시 실행하세요)")
                continue
            captured[key] = angles
            print(f"  캡처: {_fmt(angles)}  (서보 재체결됨)")

        print("\n" + "=" * 64)
        print("# pick.py 의 아래 두 상수를 이 값으로 교체:")
        if "approach" in captured:
            print(f"BASKET_APPROACH_ANGLES = {_fmt(captured['approach'])}")
        print("BASKET_DROP_ANGLES = {")
        for key, grade in (("normal", "NORMAL"), ("discard", "DISCARD")):
            if key in captured:
                print(f'    "{grade}":  {_fmt(captured[key])},')
        print("}")
        print("=" * 64)
        print("→ 이 출력을 그대로 보내주면 pick.py 에 반영해줄게.")
    finally:
        arm.focus_servos()
        arm.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
