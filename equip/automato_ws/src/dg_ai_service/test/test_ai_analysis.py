#!/usr/bin/env python3
"""RP-50  DG Control ↔ DG AI Service TCP 통합 테스트.

AI 서버(dg_ai_service)를 백그라운드 스레드에 띄우고,
DG 클라이언트(dg_control.ai_client)로 접속해
start → 결과 수신 → stop 흐름과 프레이밍을 검증한다.

실행:
  source /opt/ros/jazzy/setup.bash
  cd equip/automato_ws && source install/setup.bash
  PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pytest src/dg_ai_service/test/test_ai_analysis.py -v
"""
import socket
import threading

import pytest

from dg_ai_service.analysis_server import run_server
from dg_control.ai_client import analyze


def _free_port() -> int:
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind(('127.0.0.1', 0))
    port = s.getsockname()[1]
    s.close()
    return port


@pytest.fixture
def server():
    """AI 분석 서버를 once 모드로 백그라운드 실행, listen 준비까지 대기."""
    port = _free_port()
    ready = threading.Event()
    thread = threading.Thread(
        target=run_server,
        kwargs={'host': '127.0.0.1', 'port': port, 'once': True, 'ready': ready},
        daemon=True,
    )
    thread.start()
    assert ready.wait(timeout=3.0), '서버가 listen 상태가 안 됨'
    yield port
    thread.join(timeout=3.0)


def test_analyze_returns_result(server):
    """클라이언트가 분석 결과 dict 를 받는다."""
    result = analyze(host='127.0.0.1', port=server)
    assert isinstance(result, dict)


def test_result_has_contract_keys(server):
    """결과에 Sprint 3 계약 키(status/coord/confidence)가 모두 있다."""
    result = analyze(host='127.0.0.1', port=server)
    assert set(result.keys()) == {'status', 'coord', 'confidence'}


def test_result_values(server):
    """스텁 결과 값 검증 (status 문자열, coord 3D, confidence 0~1)."""
    result = analyze(host='127.0.0.1', port=server)
    assert isinstance(result['status'], str) and result['status']
    assert len(result['coord']) == 3
    assert 0.0 <= result['confidence'] <= 1.0


def test_framing_roundtrip():
    """framing 헬퍼 단독 왕복."""
    from dg_ai_service.framing import recv_msg, send_msg

    a, b = socket.socketpair()
    payload = {'status': 'ripe', 'coord': [1.0, 2.0, 3.0], 'confidence': 0.9}
    send_msg(a, payload)
    got = recv_msg(b)
    a.close()
    b.close()
    assert got == payload
