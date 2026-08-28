#!/usr/bin/env python3
"""pick.py 상수 티칭 툴 — 실물에서 자세들을 잡아 붙여넣기 형태로 출력.

드래그 티칭(서보 풀고 손으로 이동)으로 관측/수납/바구니 자세와 그리퍼 자세를
캡처한다. 마지막에 pick.py 상수에 바로 넣을 수 있는 코드를 출력한다.

실행 (Pi, 팔 연결·포트 비어야 함):
    sudo fuser -v /dev/ttyUSB0     # 비었는지 먼저
    python3 -m ddagi_harvest.capture_poses         # 시리얼 직결
    ARM_IP=192.168.x.x python3 -m ddagi_harvest.capture_poses   # 소켓

키:
    o  관측자세(OBSERVE_ANGLES)         s  수납/안전자세(STOW_ANGLES)
    1  NORMAL 접근(놓기 좋은 포지션)     2  NORMAL 놓기(투하)
    3  DISCARD 접근                     4  DISCARD 놓기
    g  그리퍼 자세(GRIPPER_ORI, coords의 rx,ry,rz)
    v  현재 그리퍼 값 (GRIP_THRESHOLD 참고 — 빈손 vs 토마토 걸림 비교)
    l  캡처 목록      p  붙여넣기 코드 미리보기      q  종료(서보 재체결 + 출력)

바구니 투하는 'J1 수평회전 → 접근 → 놓기' 3단계라, 접근/놓기 자세를 둘 다 잡는다.
(J1 회전은 접근 자세의 J1값을 코드가 자동 사용 — 따로 캡처 불필요)

!! 서보를 풀면 팔이 중력으로 처진다. 반드시 손으로 받친 뒤 진행.
"""
from __future__ import annotations

import os
import sys
import termios
import time
import tty


def _getch() -> str:
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def _connect():
    port = os.environ.get("ARM_PORT", "/dev/ttyUSB0")
    baud = int(os.environ.get("ARM_BAUD", "1000000"))
    ip = os.environ.get("ARM_IP")
    from pymycobot import MyCobot280
    if ip:
        from pymycobot import MyCobot280Socket
        mc = MyCobot280Socket(ip, int(os.environ.get("ARM_NETPORT", "9000")))
    else:
        mc = MyCobot280(port, baud)
    time.sleep(0.5)
    return mc


def _fmt(vals):
    return "[" + ", ".join(f"{v:.1f}" for v in vals) + "]"


def _render(captured: dict) -> str:
    a = captured.get("angles", {})

    def g(key):
        return _fmt(a[key]) if key in a else "[...]  # 미캡처"

    lines = ["# ── capture_poses.py 출력 (pick.py 상수에 붙여넣기) ──"]
    lines.append(f"OBSERVE_ANGLES = {g('observe')}")
    lines.append(f"STOW_ANGLES    = {g('stow')}")
    if "gripper_ori" in captured:
        lines.append(f"GRIPPER_ORI = {_fmt(captured['gripper_ori'])}   # coords의 rx,ry,rz")
    lines.append("BASKET_APPROACH_ANGLES = {")
    lines.append(f'    "NORMAL":  {g("normal_approach")},')
    lines.append(f'    "DISCARD": {g("discard_approach")},')
    lines.append("}")
    lines.append("BASKET_DROP_ANGLES = {")
    lines.append(f'    "NORMAL":  {g("normal_drop")},')
    lines.append(f'    "DISCARD": {g("discard_drop")},')
    lines.append("}")
    return "\n".join(lines)


def main() -> int:
    try:
        mc = _connect()
    except Exception as exc:
        print(f"연결 실패: {exc}")
        return 1

    if not mc.get_angles():
        print("각도 읽기 실패 — 포트 점유(fuser)나 baud 확인")
        return 1

    print(__doc__)
    if input("서보를 풀고 티칭을 시작할까요? (y/N) ").strip().lower() != "y":
        return 0

    mc.release_all_servos()
    print("\n서보 해제됨 — 손으로 자세를 잡고 키를 누르세요. (q=종료)\n")

    captured: dict = {"angles": {}}
    key_map = {
        "o": "observe", "s": "stow",
        "1": "normal_approach", "2": "normal_drop",
        "3": "discard_approach", "4": "discard_drop",
    }
    label = {
        "observe": "관측자세", "stow": "수납자세",
        "normal_approach": "NORMAL 접근", "normal_drop": "NORMAL 놓기",
        "discard_approach": "DISCARD 접근", "discard_drop": "DISCARD 놓기",
    }

    try:
        while True:
            k = _getch()
            if k in ("q", "\x03"):
                break
            if k in key_map:
                ang = mc.get_angles()
                if not ang:
                    print("  [실패] 각도 못 읽음, 다시"); continue
                captured["angles"][key_map[k]] = ang
                print(f"  [{label[key_map[k]]}] {_fmt(ang)}")
            elif k == "g":
                c = mc.get_coords()
                if not c:
                    print("  [실패] 좌표 못 읽음, 다시"); continue
                captured["gripper_ori"] = c[3:6]
                print(f"  [그리퍼자세] rx,ry,rz = {_fmt(c[3:6])}  (전체 coords={_fmt(c)})")
            elif k == "v":
                print(f"  [그리퍼값] {mc.get_gripper_value()}")
            elif k == "l":
                print("  --- 캡처 목록 ---")
                for name, ang in captured["angles"].items():
                    print(f"    {label[name]}: {_fmt(ang)}")
                if "gripper_ori" in captured:
                    print(f"    그리퍼자세: {_fmt(captured['gripper_ori'])}")
            elif k == "p":
                print("\n" + _render(captured) + "\n")
    finally:
        print("\n서보 재체결 — 팔을 계속 받치세요.")
        mc.focus_all_servos()
        time.sleep(1.0)

    print("\n" + "=" * 60)
    print(_render(captured))
    print("=" * 60)
    print("→ 위 값을 ddagi_harvest/pick.py 상단 상수에 붙여넣으세요.")
    print("  오프셋(PREGRASP/DESCEND/RETREAT)은 pick 실물 테스트로 튜닝합니다.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
