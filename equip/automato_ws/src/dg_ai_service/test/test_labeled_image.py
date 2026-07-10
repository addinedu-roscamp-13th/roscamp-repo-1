#!/usr/bin/env python3
"""시나리오1 E2 analyze_frame_request/response — 레이블링 이미지 반환 검증.

rotten 또는 disease 가 하나라도 감지되면 응답 result 에 레이블링된 이미지
(base64 JPEG)가 함께 담겨야 한다. 실제 YOLO 모델/모델 파일 없이도 돌 수
있도록 detector 를 페이크로 대체해 handle_analyze_frame_request 를 단위
테스트한다.

실행:
  PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pytest src/dg_ai_service/test/test_labeled_image.py -v
"""
import base64
from collections import Counter

import numpy as np

from dg_ai_service.analysis_server import handle_analyze_frame_request

JPEG_MAGIC = b'\xff\xd8'


class _FakeResult:
    """TomatoDetector.encode_labeled_image() 가 필요로 하는 최소 인터페이스."""

    def plot(self):
        return np.zeros((4, 4, 3), dtype=np.uint8)


class _FakeDetector:
    """실제 모델 대신 정해진 클래스 개수를 돌려주는 테스트용 detector."""

    def __init__(self, counts):
        self._counts = counts

    def analyze(self, image_data):
        return Counter(self._counts), _FakeResult()


def _request(request_id='req_1'):
    return {
        'message_type': 'analyze_frame_request',
        'request_id': request_id,
        'task_id': 1024,
        'waypoint_id': 3,
        'image_encoding': 'jpeg',
        'image_data': base64.b64encode(b'fake-jpeg-bytes').decode('ascii'),
    }


def test_no_labeled_image_when_only_ripe_and_unripe():
    detector = _FakeDetector({'ripe': 3, 'unripe': 2})
    response = handle_analyze_frame_request(_request(), detector)

    assert response['status'] == 'OK'
    assert 'labeled_image' not in response['result']
    assert 'labeled_image_encoding' not in response['result']


def test_labeled_image_present_when_rotten_detected():
    detector = _FakeDetector({'ripe': 1, 'rotten': 1})
    response = handle_analyze_frame_request(_request(), detector)

    assert response['status'] == 'OK'
    assert response['result']['labeled_image_encoding'] == 'jpeg'
    raw = base64.b64decode(response['result']['labeled_image'])
    assert raw[:2] == JPEG_MAGIC


def test_labeled_image_present_when_disease_detected():
    detector = _FakeDetector({'unripe': 1, 'disease': 2})
    response = handle_analyze_frame_request(_request(), detector)

    assert response['status'] == 'OK'
    assert 'labeled_image' in response['result']
    raw = base64.b64decode(response['result']['labeled_image'])
    assert raw[:2] == JPEG_MAGIC


def test_no_detections_no_labeled_image():
    detector = _FakeDetector({})
    response = handle_analyze_frame_request(_request(), detector)

    assert response['status'] == 'OK'
    assert response['result'] == {
        'ripe_percent': 0, 'unripe_percent': 0,
        'rotten_percent': 0, 'disease_percent': 0,
    }
