#!/usr/bin/env python3
"""AiTcpClient 엔드포인트 자동 전환 단위 테스트.

dg_ai_target.json 의 active("sim"->"real")를 바꾸면, 다음 analyze() 에서
DCS의 AI TCP 클라이언트가 실서버 엔드포인트로 자동 재접속하는지 검증한다.
(대시보드에서 dg_ai 시뮬을 끄면 active 가 real 로 바뀌는 시나리오의 핵심 로직.)

실행:
  PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pytest src/dg_control/test/test_ai_switch.py -v
"""
import json
import os
import socket
import struct
import tempfile
import threading
import time

import pytest

from dg_control.ai_client import AiTcpClient


def _serve(port, tag, stop):
    """포트별로 ripe_percent=tag 를 돌려주는 최소 AI 서버(어느 엔드포인트가 응답했는지 식별용)."""
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(('127.0.0.1', port))
    srv.listen(5)
    srv.settimeout(0.3)

    def handle(conn):
        try:
            while True:
                hdr = b''
                while len(hdr) < 4:
                    c = conn.recv(4 - len(hdr))
                    if not c:
                        return
                    hdr += c
                size = struct.unpack('>I', hdr)[0]
                body = b''
                while len(body) < size:
                    c = conn.recv(size - len(body))
                    if not c:
                        return
                    body += c
                req = json.loads(body.decode())
                resp = {'message_type': 'analyze_frame_response',
                        'request_id': req.get('request_id'), 'status': 'OK',
                        'result': {'ripe_percent': tag, 'unripe_percent': 0,
                                   'rotten_percent': 0, 'disease_percent': 0}}
                payload = json.dumps(resp).encode()
                conn.sendall(struct.pack('>I', len(payload)) + payload)
        except OSError:
            pass
        finally:
            conn.close()

    while not stop.is_set():
        try:
            conn, _ = srv.accept()
        except socket.timeout:
            continue
        threading.Thread(target=handle, args=(conn,), daemon=True).start()
    srv.close()


@pytest.fixture
def two_servers():
    stop = threading.Event()
    t_sim = threading.Thread(target=_serve, args=(9301, 1, stop), daemon=True)   # sim: ripe=1
    t_real = threading.Thread(target=_serve, args=(9302, 2, stop), daemon=True)  # real: ripe=2
    t_sim.start()
    t_real.start()
    time.sleep(0.4)
    yield
    stop.set()
    time.sleep(0.5)


def test_active_flag_switches_endpoint(two_servers):
    fd, path = tempfile.mkstemp(suffix='.json')
    os.close(fd)
    try:
        # 처음엔 sim 활성
        with open(path, 'w') as f:
            json.dump({'real': '127.0.0.1:9302', 'sim': '127.0.0.1:9301', 'active': 'sim'}, f)
        client = AiTcpClient(target_file=path, default_endpoint='127.0.0.1:9301')

        r1 = client.analyze('r1', 1, 0)
        assert r1['result']['ripe_percent'] == 1          # sim 서버가 응답
        assert client._cur_endpoint == '127.0.0.1:9301'

        # dg_ai 시뮬 off 흉내 → active 를 real 로 전환
        with open(path, 'w') as f:
            json.dump({'real': '127.0.0.1:9302', 'sim': '127.0.0.1:9301', 'active': 'real'}, f)

        r2 = client.analyze('r2', 1, 1)
        assert r2['result']['ripe_percent'] == 2          # 실서버로 자동 재접속됨
        assert client._cur_endpoint == '127.0.0.1:9302'
        client.close()
    finally:
        os.unlink(path)


def test_fallback_to_default_when_no_file(two_servers):
    client = AiTcpClient(target_file='/nonexistent/x.json', default_endpoint='127.0.0.1:9301')
    r = client.analyze('r', 1, 0)
    assert r['result']['ripe_percent'] == 1
    client.close()
