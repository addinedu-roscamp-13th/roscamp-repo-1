#!/usr/bin/env python3
"""시나리오1 E2  DG Control <-> DG AI Service  analyze_frame TCP 통합 테스트.

AI 서버(dg_ai_service)를 페이크 detector 로 백그라운드에 띄우고,
dg_control.ai_client.analyze_frame() 으로 요청/응답 왕복과
rotten/disease 감지 시 레이블링 이미지 동봉 여부를 검증한다.
ROS2 노드가 필요 없는 순수 TCP 테스트라 rclpy 없이도 동작한다.

실행:
  PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pytest src/dg_control/test/test_analyze_frame_client.py -v
"""
import socket
import threading
from collections import Counter

import numpy as np

from dg_ai_service.analysis_server import run_server
from dg_control.ai_client import analyze_frame, decode_labeled_image

JPEG_MAGIC = b'\xff\xd8'


class _FakeResult:
    def plot(self):
        return np.zeros((4, 4, 3), dtype=np.uint8)


class _FakeDetector:
    def __init__(self, counts):
        self._counts = counts

    def analyze(self, image_data):
        return Counter(self._counts), _FakeResult()


def _free_port() -> int:
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind(('127.0.0.1', 0))
    port = s.getsockname()[1]
    s.close()
    return port


def _start_server(counts) -> int:
    port = _free_port()
    ready = threading.Event()
    threading.Thread(
        target=run_server,
        kwargs={
            'host': '127.0.0.1', 'port': port, 'once': True, 'ready': ready,
            'detector': _FakeDetector(counts),
        },
        daemon=True,
    ).start()
    assert ready.wait(timeout=3.0), '서버가 listen 상태가 안 됨'
    return port


def test_analyze_frame_returns_percentages_without_labeled_image():
    port = _start_server({'ripe': 4, 'unripe': 1})
    result = analyze_frame(
        b'fake-jpeg-bytes', task_id=1024, waypoint_id=3,
        host='127.0.0.1', port=port,
    )

    assert result['ripe_percent'] == 80
    assert result['unripe_percent'] == 20
    assert 'labeled_image' not in result


def test_analyze_frame_returns_labeled_image_on_rotten():
    port = _start_server({'ripe': 1, 'rotten': 1})
    result = analyze_frame(
        b'fake-jpeg-bytes', task_id=1024, waypoint_id=3,
        host='127.0.0.1', port=port,
    )

    image_bytes = decode_labeled_image(result)
    assert image_bytes is not None
    assert image_bytes[:2] == JPEG_MAGIC


def test_analyze_frame_returns_labeled_image_on_disease():
    port = _start_server({'unripe': 2, 'disease': 1})
    result = analyze_frame(
        b'fake-jpeg-bytes', task_id=1024, waypoint_id=3,
        host='127.0.0.1', port=port,
    )

    image_bytes = decode_labeled_image(result)
    assert image_bytes is not None
    assert image_bytes[:2] == JPEG_MAGIC


def test_decode_labeled_image_returns_none_when_absent():
    assert decode_labeled_image({'ripe_percent': 100}) is None
