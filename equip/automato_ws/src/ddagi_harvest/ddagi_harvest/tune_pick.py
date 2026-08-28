#!/usr/bin/env python3
"""파지 오프셋 튜닝 + 실물 파지 테스트 (standalone, pymycobot만 필요).

접근/후퇴 오프셋을 추측하지 않고 **드래그로 3점을 잡아 자동 산출**한다:
    grasp     : 그리퍼가 토마토를 집는 바로 그 자세 (target + 그리퍼자세)
    pre-grasp : 접근 시작(뒤로/위로 물러난) 자세
    retreat   : 집고 들어올린 자세
→ PREGRASP_OFFSET = pre-grasp − grasp,  RETREAT_OFFSET = retreat − grasp,
  GRIPPER_ORI = grasp 자세(rx,ry,rz).  이 값을 pick.py에 붙여넣는다.

그다음 't'로 그 값으로 실제 파지 시퀀스(파지까지)를 돌려 검증한다.

실행 (Pi, venv):
    source ~/venv/automato/bin/activate
    sudo fuser -v /dev/ttyUSB0
    python3 tune_pick.py

키:
    g  grasp 자세 캡처(드래그)     p  pre-grasp 캡처     r  retreat 캡처
    v  현재 그리퍼값 (임계값 참고)   d  산출된 오프셋 출력
    c  처짐 보정 입력 (dx dy dz mm) — 서보로 실행 시 팔이 쳐지는 만큼 반대로
    t  이 값으로 실제 파지 테스트(서보 ON, 확인 후 실행)
    q  종료(서보 재체결)

처짐(sag): 캡처는 서보 풀고 하지만 실행은 서보 켜고 해서, 중력으로 팔이 몇 mm
쳐진다(알려진 이슈). 'c'로 반대 방향 보정을 넣어 't'로 맞춘다. 아래로 쳐지면 +z.

!! 캡처는 서보를 풀어 손으로(팔 받치기). 테스트(t)는 서보 켜고 자동 이동.
좌표 mm / 각도 deg.
"""
from __future__ import annotations

import json
import os
import sys
import termios
import time
import tty

APPROACH_SPEED = 30
RETREAT_SPEED = 30
GRIP_THRESHOLD = 10
PTS_FILE = "taught_pick.json"   # 캡처 저장(재시작해도 유지). 초기화하려면 삭제.


def _save(pts):
    try:
        with open(PTS_FILE, "w") as f:
            json.dump(pts, f)
    except Exception:
        pass


def _load():
    try:
        with open(PTS_FILE) as f:
            return json.load(f)
    except Exception:
        return {}


def _getch():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def _connect():
    ip = os.environ.get("ARM_IP")
    from pymycobot import MyCobot280
    if ip:
        from pymycobot import MyCobot280Socket
        mc = MyCobot280Socket(ip, int(os.environ.get("ARM_NETPORT", "9000")))
    else:
        mc = MyCobot280(os.environ.get("ARM_PORT", "/dev/ttyUSB0"),
                        int(os.environ.get("ARM_BAUD", "1000000")))
    time.sleep(0.5)
    return mc


def _fmt(v):
    return "[" + ", ".join(f"{x:.1f}" for x in v) + "]"


def _sub(a, b):
    return [a[i] - b[i] for i in range(3)]


def _capture(mc, name):
    """서보 풀고 손으로 자세 잡게 한 뒤 get_coords 캡처."""
    print(f"  '{name}' 자세로 팔을 옮기세요(서보 해제). 준비되면 Enter, 취소 c.")
    mc.release_all_servos()
    k = _getch()
    if k in ("c", "\x03"):
        mc.focus_all_servos()
        return None
    c = mc.get_coords()
    mc.focus_all_servos()
    time.sleep(0.5)
    if not c:
        print("  [실패] 좌표 못 읽음")
        return None
    print(f"  {name} = {_fmt(c)}  (서보 재체결됨)")
    return c


def _offsets(pts):
    g = pts.get("grasp")
    out = {}
    if g:
        out["GRIPPER_ORI"] = g[3:6]
        if "pregrasp" in pts:
            out["PREGRASP_OFFSET"] = _sub(pts["pregrasp"], g)
        if "retreat" in pts:
            out["RETREAT_OFFSET"] = _sub(pts["retreat"], g)
    return out


def _render(pts):
    o = _offsets(pts)
    lines = ["# ── tune_pick.py 산출 (pick.py 상수에 붙여넣기) ──"]
    lines.append(f"GRIPPER_ORI = {_fmt(o['GRIPPER_ORI']) if 'GRIPPER_ORI' in o else '[...]  # grasp 미캡처'}")
    lines.append(f"PREGRASP_OFFSET = {_fmt(o['PREGRASP_OFFSET']) if 'PREGRASP_OFFSET' in o else '[...]  # pre-grasp 미캡처'}")
    sag = pts.get("sag")
    lines.append(f"DESCEND_OFFSET = {_fmt(sag) if sag else '[0.0, 0.0, 0.0]'}   # 처짐 보정 (아래로 쳐지면 +z, 오른쪽이면 좌로)")
    lines.append(f"RETREAT_OFFSET = {_fmt(o['RETREAT_OFFSET']) if 'RETREAT_OFFSET' in o else '[...]  # retreat 미캡처'}")
    return "\n".join(lines)


def _test(mc, pts):
    g = pts.get("grasp")
    if not g:
        print("  grasp 미캡처 — 먼저 'g'로 잡으세요.")
        return
    o = _offsets(pts)
    ori = g[3:6]
    sag = pts.get("sag", [0.0, 0.0, 0.0])
    target = [g[i] + sag[i] for i in range(3)]   # 처짐 보정된 목표
    pre = [target[i] + o.get("PREGRASP_OFFSET", [0, 0, 60])[i] for i in range(3)] + ori
    grasp = list(target) + ori
    ret = [target[i] + o.get("RETREAT_OFFSET", [0, 0, 93])[i] for i in range(3)] + ori

    print(f"\n  [테스트] 처짐보정 sag={_fmt(sag)}, 서보 ON, 아래 순서로 자동 이동:")
    print(f"    pre-grasp {_fmt(pre)}\n    grasp {_fmt(grasp)}\n    (close)\n    retreat {_fmt(ret)}")
    print("  팔 반경 확보 확인. 진행? (y/N) ", end="", flush=True)
    if _getch().lower() != "y":
        print("\n  취소")
        return
    print()
    to = int(os.environ.get("MOVE_TIMEOUT", "15"))
    mc.set_gripper_value(100, 50); time.sleep(1.5)          # 열기
    mc.sync_send_coords(pre, APPROACH_SPEED, 1, timeout=to)  # 직선 접근 시작
    mc.sync_send_coords(grasp, APPROACH_SPEED, 1, timeout=to)
    mc.set_gripper_value(0, 50); time.sleep(1.5)            # 닫기
    val = mc.get_gripper_value()
    grabbed = val > GRIP_THRESHOLD
    mc.sync_send_coords(ret, RETREAT_SPEED, 1, timeout=to)  # 후퇴
    print(f"  결과: 그리퍼값={val} → {'파지 성공' if grabbed else '실패(빈 그리퍼)'}")


def main():
    try:
        mc = _connect()
    except Exception as exc:
        print(f"연결 실패: {exc}")
        return 1
    if not mc.get_coords():
        print("좌표 읽기 실패 — 포트 점유/baud 확인")
        return 1

    print(__doc__)
    pts = _load()
    if pts:
        have = [k for k in ("grasp", "pregrasp", "retreat") if k in pts]
        print(f"이전 캡처 로드됨: {', '.join(have) or '없음'}"
              f"{'  sag=' + _fmt(pts['sag']) if 'sag' in pts else ''}")
    cap_keys = {"g": "grasp", "p": "pregrasp", "r": "retreat"}
    try:
        while True:
            print("\n키(g/p/r 캡처, v 그리퍼값, c 처짐보정, d 오프셋, t 테스트, q 종료): ",
                  end="", flush=True)
            k = _getch()
            print(k)
            if k in ("q", "\x03"):
                break
            if k in cap_keys:
                c = _capture(mc, cap_keys[k])
                if c:
                    pts[cap_keys[k]] = c
                    _save(pts)
            elif k == "v":
                print(f"  그리퍼값 = {mc.get_gripper_value()}")
            elif k == "c":
                cur = pts.get("sag", [0.0, 0.0, 0.0])
                try:
                    raw = input(f"  처짐 보정 dx dy dz mm (현재 {_fmt(cur)}, 예: 0 0 6): ").split()
                    pts["sag"] = [float(v) for v in raw]
                    _save(pts)
                    print(f"  보정 = {_fmt(pts['sag'])}")
                except (ValueError, IndexError):
                    print("  입력 형식 오류 (숫자 3개: dx dy dz)")
            elif k == "d":
                print("\n" + _render(pts))
            elif k == "t":
                _test(mc, pts)
    finally:
        mc.focus_all_servos()
    print("\n" + "=" * 60 + "\n" + _render(pts) + "\n" + "=" * 60)
    return 0


if __name__ == "__main__":
    sys.exit(main())
