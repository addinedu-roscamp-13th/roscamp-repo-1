#!/usr/bin/env python3
"""팔 서버 (Pi에서 실행) — 노트북이 팔을 네트워크로 조종하게 하는 다리.

노트북의 NetworkArm(클라이언트)이 보낸 명령을 받아 pymycobot으로 실행하고
결과를 돌려준다. 개발 편의용(카메라가 노트북에 있어 노트북에서 다 돌리려고).
최종 배포는 Pi 로컬(RealArm)이라 이 서버는 개발 단계만 쓴다.

프로토콜: 한 줄 = JSON 요청 {"m": 메서드, "a": [인자...]} → 응답 {"ok":..,"r":..}
        (줄바꿈 구분)

실행 (Pi, venv, 포트 비어야 함):
    source ~/venv/automato/bin/activate
    python3 arm_server.py                 # 기본 0.0.0.0:9010
"""
from __future__ import annotations

import json
import os
import socketserver
import time

from pymycobot import MyCobot280

PORT = int(os.environ.get("ARM_SERVER_PORT", "9010"))
_mc = None


def get_mc():
    global _mc
    if _mc is None:
        _mc = MyCobot280(os.environ.get("ARM_PORT", "/dev/ttyUSB0"),
                         int(os.environ.get("ARM_BAUD", "1000000")))
        time.sleep(0.5)
    return _mc


def dispatch(method: str, args: list):
    """허용된 메서드만 pymycobot으로 전달 (화이트리스트)."""
    mc = get_mc()
    allowed = {
        "get_coords", "get_angles", "get_gripper_value", "is_gripper_moving",
        "send_coords", "send_angles", "sync_send_coords", "sync_send_angles",
        "set_gripper_value", "release_all_servos", "focus_all_servos", "stop",
        "set_gripper_calibration", "set_gripper_mode", "get_gripper_mode",
        # 보호 전류(1~500) — 파지력 상한. 발표용 수치 인용은 get 으로 읽어서.
        # ⚠ set 은 파지값 스케일을 바꿔 GRIP_THRESHOLD 재튜닝이 필요하니 신중히.
        "get_gripper_protect_current", "set_gripper_protect_current",
    }
    if method not in allowed:
        raise ValueError(f"허용되지 않은 메서드: {method}")
    return getattr(mc, method)(*args)


class Handler(socketserver.StreamRequestHandler):
    def handle(self):
        peer = self.client_address[0]
        print(f"[연결] {peer}")
        for line in self.rfile:
            try:
                req = json.loads(line.decode().strip())
                r = dispatch(req["m"], req.get("a", []))
                resp = {"ok": True, "r": r}
            except Exception as exc:
                resp = {"ok": False, "err": str(exc)}
                print(f"[에러] {exc}")
            self.wfile.write((json.dumps(resp) + "\n").encode())
            self.wfile.flush()
        print(f"[해제] {peer}")


class Server(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True


def main():
    get_mc()  # 시작 시 시리얼 연결 확인
    with Server(("0.0.0.0", PORT), Handler) as srv:
        print(f"팔 서버 시작 :{PORT}  (Ctrl+C 종료)")
        try:
            srv.serve_forever()
        except KeyboardInterrupt:
            print("\n종료")


if __name__ == "__main__":
    main()
