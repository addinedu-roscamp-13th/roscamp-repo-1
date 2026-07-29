#!/usr/bin/env python3
"""예냉실 하역(Unload) 모션 티칭 & 재생 — 시나리오2 E6.

바구니 손잡이를 잡아 들어올리고, 예냉실 위에서 기울여 쏟고, 털어낸 뒤 복귀하는
동작을 손으로 끌어 가르친다. 좌표가 아니라 **관절각**으로 저장한다 — 같은 좌표라도
IK 해가 매번 달라져 경로가 튀는 문제를 피하려면 관절각 재생이 안전하다(반복 테스트
단계에서 확인된 결론).

    티칭:  python3 ddagi_harvest/teach_unload.py teach
    확인:  python3 ddagi_harvest/teach_unload.py show
    검사:  python3 ddagi_harvest/teach_unload.py check
    재생:  python3 ddagi_harvest/teach_unload.py run

티칭 키:
    SPACE  현재 자세를 웨이포인트로 저장 (그리퍼 동작 없음)
    g      저장 + 여기서 그리퍼 닫기   (= 바구니 손잡이 파지)
    o      저장 + 여기서 그리퍼 열기   (= 손잡이 놓기)
    w      저장 + 여기서 대기          (기울인 채 쏟아지길 기다림)
    s      저장 + 여기서 털기          (손목 관절 왕복)
    u      마지막 기록 취소
    l      지금까지 기록 목록
    q      기록 종료(저장)

■ 반드시 바구니를 매단 채로 티칭할 것
서보를 풀고 손으로 끌어 가르치지만 재생은 서보를 켜고 한다. 이 차이로 팔이 중력에
더 숙여지는데(반복 테스트에서 확인된 처짐), **짐을 든 상태에서는 그 처짐이 더 커진다.**
빈 팔로 가르친 자세로 바구니를 들면 예냉실 입구보다 낮게 도달할 수 있다. 티칭할 때
바구니를 걸어두면 자세가 기하학적으로 맞고, 재생 후 실제 도달 높이만 확인하면 된다.

■ 그리퍼는 티칭 중에 닫아둘 것
'g' 는 재생 시점에 닫으라는 표시일 뿐, 티칭 중에는 손잡이를 물고 있어야 이후 자세가
바구니 무게를 반영한다.
"""
from __future__ import annotations

import json
import os
import sys
import termios
import time
import tty

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from ddagi_harvest.arm_backend import NetworkArm      # noqa: E402

PATH_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                         "unload_path.json")

# 재생 속도 — 짐을 들고 움직이므로 파지 속도(30)보다도 낮게 시작한다.
SPEED = 25
SETTLE = 0.4              # 스텝 후 정착 대기(s)
GRIPPER_SPEED = 60
WAIT_SEC = 3.0            # 'w' 스텝 기본 대기 (Unload.action 의 shake_delay_sec)

# ---- 털기 ------------------------------------------------------------------ #
# 손목 관절 하나를 ±진폭으로 왕복시킨다. 좌표로 흔들면 왕복마다 IK 해가 달라져
# 경로가 튀므로 관절각으로만 흔든다.
#   J5(손목 피치) = 바구니를 위아래로 까딱 → 남은 열매가 굴러 나온다
#   J6(손목 회전) = 비틀기. 손잡이가 하나뿐이라 바구니가 돌아갈 수 있어 권하지 않는다
SHAKE_JOINT = 5           # 1-indexed (J5)
SHAKE_AMPLITUDE = 8.0     # ±도. 작게 시작할 것 — 크면 그리퍼가 손잡이를 놓친다
SHAKE_CYCLES = 3
SHAKE_SPEED = 55          # 이동보다 빠르게(털어야 하므로) 그러나 최대치는 피한다

# 반복 테스트에서 검증된 명령 한계(arm_util.py). 드래그 티칭은 이보다 넓게 꺾인다 —
# 손으로 J3 를 155° 까지 끌 수 있지만 명령은 150° 에서 거부된다.
JOINT_LIMITS = {1: (-168, 168), 2: (-140, 140), 3: (-150, 150),
                4: (-150, 150), 5: (-155, 160), 6: (-180, 180)}
CLAMP_MARGIN = 0.5

ARM_IP = os.environ.get("ARM_IP", "192.168.100.12")

_ACT_LABEL = {"grip": "손잡이 파지", "open": "손잡이 놓기",
              "wait": f"{WAIT_SEC:.0f}초 대기", "shake": "털기"}


def clamp_angles(angles):
    """관절각을 명령 가능 한계 안으로 조인다. (clamped, changes) 반환."""
    clamped, changes = [], []
    for i, a in enumerate(angles, start=1):
        lo, hi = JOINT_LIMITS[i]
        c = min(max(a, lo + CLAMP_MARGIN), hi - CLAMP_MARGIN)
        if abs(c - a) > 1e-6:
            changes.append((i, a, c))
        clamped.append(c)
    return clamped, changes


def read_stable(arm, tries: int = 3, tol: float = 0.6, gap: float = 0.25):
    """같은 값이 연속으로 읽힐 때까지 읽어 '정지 확정' 각도를 돌려준다.

    움직인 직후 1회 읽기는 못 믿는다 — 반복 테스트에서 J6 가 -53.43° 로 읽혔다가
    실제로는 -0.43° 였던 사건이 있었다. 연속 일치를 봐야 진짜 멈춘 값이다.
    """
    prev = None
    for _ in range(tries * 3):
        cur = arm.get_angles()
        if cur and prev and all(abs(a - b) < tol for a, b in zip(cur, prev)):
            return cur
        prev = cur
        time.sleep(gap)
    return prev


def getch() -> str:
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def phase_of(step: dict, idx: int, steps: list) -> str:
    """Unload.action 의 phase 로 매핑한다(나중에 액션 서버가 그대로 쓴다).

    GRIP_HANDLE / LIFT / WAIT / SHAKE / RETURN
    """
    act = step.get("act")
    if act == "grip":
        return "GRIP_HANDLE"
    if act == "wait":
        return "WAIT"
    if act == "shake":
        return "SHAKE"
    if act == "open":
        return "RETURN"
    gripped = any(s.get("act") == "grip" for s in steps[:idx])
    released = any(s.get("act") == "open" for s in steps[:idx])
    if released:
        return "RETURN"
    return "LIFT" if gripped else "GRIP_HANDLE"


def describe(steps: list) -> None:
    for i, s in enumerate(steps, start=1):
        act = s.get("act")
        tail = f"   ← {_ACT_LABEL[act]}" if act else ""
        print(f"  {i:2d}. [{phase_of(s, i - 1, steps):<11}] "
              f"{[round(a, 1) for a in s['angles']]}{tail}")


# ---- 티칭 ------------------------------------------------------------------ #

def do_teach(arm) -> None:
    cur = read_stable(arm)
    if not cur:
        raise SystemExit("각도 읽기 실패 — Pi 의 arm_server.py 가 떠 있는지 확인")
    print("현재 자세:", [round(a, 1) for a in cur])
    print(__doc__.split("티칭 키:")[1].split("■ 반드시")[0])
    print("!! 서보를 풀면 팔이 중력으로 처집니다. 반드시 팔을 손으로 받친 뒤 진행하세요.")
    print("!! 바구니를 매단 채로 가르치세요 — 짐 무게가 자세에 반영돼야 합니다.")
    if input("서보를 풀고 티칭을 시작할까요? (y/N) ").strip().lower() != "y":
        raise SystemExit("취소됨")

    arm.release_servos()
    print("\n서보 해제됨 — 손으로 팔을 원하는 자세로 옮기고 키를 누르세요.\n")

    steps: list[dict] = []
    try:
        while True:
            key = getch()
            if key in ("q", "\r", "\n", "\x03"):
                break
            if key == "u":
                if steps:
                    steps.pop()
                    print(f"  취소됨 — 남은 기록 {len(steps)}개")
                continue
            if key == "l":
                print("  --- 기록 목록 ---")
                describe(steps)
                continue
            if key not in (" ", "g", "o", "w", "s"):
                continue
            angles = read_stable(arm, tries=2, tol=1.5, gap=0.2)
            if not angles:
                print("  [실패] 각도를 읽지 못했습니다. 다시 시도하세요.")
                continue
            act = {"g": "grip", "o": "open", "w": "wait", "s": "shake"}.get(key)
            steps.append({"angles": angles, "act": act})
            tail = f" + {_ACT_LABEL[act]}" if act else ""
            print(f"  [{len(steps):2d}] 기록{tail}: {[round(a, 1) for a in angles]}")
    finally:
        print("\n서보를 다시 켭니다 — 팔을 계속 받치고 있으세요.")
        arm.focus_servos()
        time.sleep(1.0)

    if not steps:
        raise SystemExit("기록된 자세가 없습니다.")
    with open(PATH_FILE, "w", encoding="utf-8") as fp:
        json.dump({"speed": SPEED, "shake": {"joint": SHAKE_JOINT,
                                             "amplitude": SHAKE_AMPLITUDE,
                                             "cycles": SHAKE_CYCLES},
                   "steps": steps}, fp, ensure_ascii=False, indent=2)
    print(f"\n총 {len(steps)}개 자세를 저장했습니다 → {PATH_FILE}")
    describe(steps)
    print(f"\n검사:  python3 {os.path.basename(__file__)} check")
    print(f"재생:  python3 {os.path.basename(__file__)} run")


# ---- 재생 ------------------------------------------------------------------ #

def load() -> dict:
    if not os.path.exists(PATH_FILE):
        raise SystemExit(f"기록 파일이 없습니다: {PATH_FILE}\n먼저 teach 를 실행하세요.")
    with open(PATH_FILE, encoding="utf-8") as fp:
        return json.load(fp)


def do_check() -> None:
    steps = load()["steps"]
    bad = False
    for i, s in enumerate(steps, start=1):
        _, changes = clamp_angles(s["angles"])
        if changes:
            bad = True
            note = ", ".join(f"J{j}={o:.1f}(초과→{c:.1f})" for j, o, c in changes)
            warn = "  * 그리퍼·털기 스텝 — 재티칭 권장" if s.get("act") else ""
            print(f"  step {i}: {note}{warn}")
    print("모든 스텝이 명령 한계 안에 있습니다. 이상 없음." if not bad
          else "\n한계를 넘는 스텝이 있습니다. 재생 시 자동으로 조여지지만, "
               "동작이 의도와 달라질 수 있으니 재티칭을 권합니다.")


def do_shake(arm, base_angles, cfg: dict) -> None:
    """손목 관절을 ±진폭으로 왕복시켜 남은 열매를 털어낸다.

    그리퍼는 건드리지 않는다 — 흔드는 중에 손잡이를 놓으면 바구니가 떨어진다.
    진폭을 키우기 전에 반드시 낮은 값으로 확인할 것.
    """
    j = int(cfg.get("joint", SHAKE_JOINT)) - 1        # 0-indexed
    amp = float(cfg.get("amplitude", SHAKE_AMPLITUDE))
    cycles = int(cfg.get("cycles", SHAKE_CYCLES))
    print(f"    털기: J{j + 1} ±{amp:.1f}° × {cycles}회")
    for c in range(cycles):
        for sign in (+1, -1):
            a = list(base_angles)
            a[j] += sign * amp
            a, _ = clamp_angles(a)
            arm.move_angles(a, SHAKE_SPEED)
        print(f"      {c + 1}/{cycles}")
    arm.move_angles(clamp_angles(base_angles)[0], SHAKE_SPEED)   # 원 자세로


def do_run(arm) -> None:
    data = load()
    steps, shake_cfg = data["steps"], data.get("shake", {})
    speed = data.get("speed", SPEED)
    print(f"기록된 자세 {len(steps)}개를 재생합니다 (속도 {speed}).")
    describe(steps)
    print("\n!! 바구니를 걸고, 팔 반경의 장애물을 치우세요. 중단은 Ctrl+C.")
    if input("시작할까요? (y/N) ").strip().lower() != "y":
        raise SystemExit("취소됨")

    t0 = time.time()
    try:
        for i, s in enumerate(steps, start=1):
            angles, changes = clamp_angles(s["angles"])
            if changes:
                note = ", ".join(f"J{j}:{o:.1f}→{c:.1f}" for j, o, c in changes)
                print(f"    [clamp] step {i} 한계초과 조정: {note}")
            ph = phase_of(s, i - 1, steps)
            act = s.get("act")
            print(f"  step {i}/{len(steps)} [{ph}]"
                  + (f" — {_ACT_LABEL[act]}" if act else ""))
            arm.move_angles(angles, speed)
            time.sleep(SETTLE)

            if act == "grip":
                arm.close_gripper(GRIPPER_SPEED)
                v = arm.gripper_value()
                print(f"    그리퍼값 {v} — "
                      + ("손잡이 물림" if v and v > 6 else "⚠ 빈손일 수 있음"))
            elif act == "open":
                arm.open_gripper(GRIPPER_SPEED)
            elif act == "wait":
                print(f"    {WAIT_SEC:.0f}초 대기(쏟아지는 중)")
                time.sleep(WAIT_SEC)
            elif act == "shake":
                do_shake(arm, angles, shake_cfg)
    except KeyboardInterrupt:
        print("\n[중단] 사용자 중지 — 팔을 그 자리에 세웁니다.")
    print(f"\n완료 ({time.time() - t0:.1f}s)")


def main() -> int:
    mode = sys.argv[1] if len(sys.argv) > 1 else ""
    if mode == "show":
        describe(load()["steps"])
        return 0
    if mode == "check":
        do_check()
        return 0
    if mode not in ("teach", "run"):
        print(__doc__)
        return 1
    print(f"팔 연결 {ARM_IP}:9010")
    arm = NetworkArm(ARM_IP)
    try:
        (do_teach if mode == "teach" else do_run)(arm)
    finally:
        arm.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
