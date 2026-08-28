#!/usr/bin/env python3
"""이름 붙은 자세로 팔을 보낸다 — 테스트 사이에 팔 자세를 되돌릴 때.

수확·하역 테스트를 반복하면 팔이 직전 테스트가 끝난 자리(예: 하역 1번 웨이포인트)에
남는다. 그 상태로 다음 테스트를 시작하면 출발 자세가 달라 결과를 비교할 수 없다.

    python3 ddagi_harvest/goto_pose.py              # 관측 자세 (기본)
    python3 ddagi_harvest/goto_pose.py --list       # 갈 수 있는 자세 목록
    python3 ddagi_harvest/goto_pose.py staging
    python3 ddagi_harvest/goto_pose.py unload1 -y   # 확인 없이 바로

■ 왜 그냥 send_angles 를 치지 않는가
관절 이동은 IK 를 안 타므로 목표는 확실하지만 **경로는 여전히 직선이 아니다.**
큰 각도를 한 번에 주면 팔이 베드·예냉실·바구니를 훑고 지나갈 수 있다. 그래서
이동 전에 관절별 차이를 보여주고, 큰 이동은 자동으로 감속한다.

■ 그리퍼는 열고 간다
여기 있는 자세들은 전부 '무언가를 잡기 직전' 이거나 '아무것도 안 든' 상태다 —
관측은 검출만 하고, 하역 1번은 손잡이를 잡으러 가는 자리다. 닫고 갈 이유가 없고,
닫힌 채로 도착하면 다음 동작 전에 결국 열어야 한다. 이동 **전에** 여는 이유는
그리퍼가 고장났을 때 큰 이동을 하기 전에 알아채기 위해서다.

예외는 하나 — **무언가 물고 있을 때**(바구니를 든 채 하역을 취소한 경우 등)는 열지
않는다. 그대로 열면 들고 있던 것이 그 자리에서 떨어진다. 경고하고 물어본다.
"""
from __future__ import annotations

import argparse
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from ddagi_harvest import pick as pk                    # noqa: E402
from ddagi_harvest.arm_backend import NetworkArm        # noqa: E402

ARM_IP = os.environ.get("ARM_IP", "192.168.3.12")

# 큰 이동일수록 느리게. 팔이 훑고 갈 여지가 그만큼 크다.
SPEED_NORMAL = 30
SPEED_SLOW = 15
SLOW_OVER_DEG = 25.0      # teach_unload.FIRST_MOVE_WARN_DEG 와 같은 기준
GRIP_HOLDING = 6          # pick.GRIP_THRESHOLD 와 같은 임계
GRIPPER_SPEED = 60


def _unload_wp1():
    """하역 티칭 경로의 1번 웨이포인트. 티칭 안 됐으면 목록에서 뺀다."""
    try:
        from ddagi_harvest import teach_unload as tu
        return tu.load()["steps"][0]["angles"]
    except BaseException:
        return None


def poses() -> dict:
    p = {
        "observe": (pk.OBSERVE_ANGLES, "관측 자세 — 검출 시 카메라가 베드를 보는 자세"),
        "staging": (pk.STAGING_ANGLES, "수확 준비 자세"),
        "basket": (pk.BASKET_APPROACH_ANGLES, "바구니 접근 자세"),
    }
    wp1 = _unload_wp1()
    if wp1:
        p["unload1"] = (wp1, "하역 1번 웨이포인트 — 손잡이 접근")
    return p


def main() -> int:
    ps = poses()
    ap = argparse.ArgumentParser(description="팔을 이름 붙은 자세로 보낸다")
    ap.add_argument("pose", nargs="?", default="observe", choices=list(ps))
    ap.add_argument("--list", action="store_true", help="자세 목록만 출력")
    ap.add_argument("-y", "--yes", action="store_true", help="확인 없이 바로 이동")
    ap.add_argument("--speed", type=int, default=0, help="속도 지정(0=자동)")
    ap.add_argument("--keep-grip", action="store_true",
                    help="그리퍼를 열지 않고 현재 상태 유지")
    ap.add_argument("--drop", action="store_true",
                    help="물고 있어도 확인 없이 연다(떨어뜨림)")
    ap.add_argument("--ip", default=ARM_IP)
    a = ap.parse_args()

    if a.list:
        for name, (ang, desc) in ps.items():
            print(f"  {name:9s} {desc}")
            print(f"            {[round(x, 1) for x in ang]}")
        return 0

    target, desc = ps[a.pose]
    print(f"목표: {a.pose} — {desc}")
    print(f"      {[round(x, 1) for x in target]}")

    arm = NetworkArm(a.ip)
    try:
        cur = arm.get_angles()
        if not cur:
            print("현재 각도를 읽지 못했습니다 — 팔 상태를 확인하세요.", file=sys.stderr)
            return 1
        print(f"현재: {[round(x, 1) for x in cur]}")

        diffs = [(i + 1, abs(c - t)) for i, (c, t) in enumerate(zip(cur, target))]
        worst = max(diffs, key=lambda d: d[1])
        big = [f"J{j} {d:.0f}°" for j, d in diffs if d > 5]
        print(f"차이: 최대 J{worst[0]} {worst[1]:.1f}°"
              + (f"  ({', '.join(big)})" if big else "  (거의 제자리)"))

        speed = a.speed or (SPEED_SLOW if worst[1] > SLOW_OVER_DEG else SPEED_NORMAL)
        if worst[1] > SLOW_OVER_DEG:
            print(f"  ⚠ 이동이 큽니다. 그 사이 경로는 팔이 정하므로 베드·예냉실·"
                  f"바구니를 훑을 수 있습니다 → 속도 {speed} 로 갑니다. 손을 대고 계세요.")

        # 이동 전에 그리퍼를 연다. 물고 있으면 떨어뜨리게 되므로 그때만 확인을 받는다.
        v = arm.gripper_value()
        holding = bool(v) and v > GRIP_HOLDING
        if a.keep_grip:
            print(f"  그리퍼 그대로 둡니다(--keep-grip, 현재 값 {v}).")
        elif holding:
            print(f"  ⚠ 그리퍼가 무언가 물고 있습니다(값 {v}) — 바구니일 수 있습니다.")
            print("     지금 열면 그 자리에서 떨어집니다.")
            if a.yes:
                # -y 는 '확인 생략' 이지 '떨어뜨려도 된다' 가 아니다.
                print("     -y 가 있어도 임의로 놓지 않습니다 → 그대로 들고 이동합니다.")
                print("     놓으려면 --drop, 유지하려면 --keep-grip 을 주세요.")
            elif a.drop or input("     열까요? (y/N) ").strip().lower() == "y":
                arm.open_gripper(GRIPPER_SPEED)
                time.sleep(0.4)
                print("     그리퍼 열었습니다.")
            else:
                print("     그대로 들고 이동합니다.")

        if not a.yes and input("이동할까요? (y/N) ").strip().lower() != "y":
            print("취소됨")
            return 1

        if not a.keep_grip and not holding:
            # 빈손이면 묻지 않고 연다. 큰 이동 전에 열어 그리퍼 고장을 먼저 드러낸다.
            arm.open_gripper(GRIPPER_SPEED)
            time.sleep(0.4)
            print("그리퍼 열었습니다.")

        t0 = time.time()
        arm.move_angles(target, speed)
        got = arm.get_angles()
        print(f"완료 ({time.time() - t0:.1f}s)")
        if got:
            err = max(abs(g - t) for g, t in zip(got, target))
            print(f"도달: {[round(x, 1) for x in got]}  (최대 오차 {err:.1f}°)")
    except KeyboardInterrupt:
        print("\n[중단] 팔을 그 자리에 세웁니다.")
        return 1
    finally:
        arm.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
