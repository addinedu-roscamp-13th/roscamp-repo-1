#!/usr/bin/env python3
"""RP-127  detect_tomatoes_server.build_tomatoes() — 등급 매핑 + depth 조회 검증.

detect_instances()의 YOLO bbox 파싱 자체는 test_detect_instances.py가 이미
검증하므로, 여기선 그 출력을 받아 등급을 매기고 3D 좌표를 붙이는 순수 로직만
가짜 detector(detect_instances가 고정 목록을 돌려주는 스텁)로 검증한다.
rclpy/실 모델/실 카메라가 전혀 필요 없다.

실행:
  PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pytest src/dg_ai_service/test/test_detect_tomatoes.py -v
"""
import numpy as np

from dg_ai_service.tomato_grading import build_tomatoes

INTRINSICS = (100.0, 100.0, 50.0, 50.0)  # fx, fy, ppx, ppy


class _FakeDetector:
    def __init__(self, detections):
        self._detections = detections

    def detect_instances(self, img, padding_px):
        return self._detections


def _det(label, u=50, v=50, confidence=0.9, radius_px=10):
    return {'label': label, 'confidence': confidence, 'center_u': u, 'center_v': v, 'radius_px': radius_px}


def test_build_tomatoes_maps_ripe_to_normal():
    detector = _FakeDetector([_det('ripe')])
    depth = np.full((100, 100), 500.0)  # 500mm = 0.5m, 중심 픽셀 = 주점

    tomatoes = build_tomatoes(detector, np.zeros((100, 100, 3), dtype=np.uint8), depth, INTRINSICS)

    assert len(tomatoes) == 1
    t = tomatoes[0]
    assert t == {'tomato_id': 0, 'grade': 'NORMAL', 'x': 0.0, 'y': 0.0, 'z': 0.5}


def test_build_tomatoes_maps_rotten_and_disease_to_discard():
    detector = _FakeDetector([_det('rotten', u=10, v=10), _det('disease', u=90, v=90)])
    depth = np.full((100, 100), 300.0)

    tomatoes = build_tomatoes(detector, np.zeros((100, 100, 3), dtype=np.uint8), depth, INTRINSICS)

    assert [t['grade'] for t in tomatoes] == ['DISCARD', 'DISCARD']
    assert [t['tomato_id'] for t in tomatoes] == [0, 1]


def test_build_tomatoes_excludes_unripe():
    detector = _FakeDetector([_det('unripe')])
    depth = np.full((100, 100), 400.0)

    tomatoes = build_tomatoes(detector, np.zeros((100, 100, 3), dtype=np.uint8), depth, INTRINSICS)

    assert tomatoes == []


def test_build_tomatoes_skips_detection_with_no_valid_depth():
    detector = _FakeDetector([_det('ripe')])
    depth = np.zeros((100, 100))  # depth 전부 무효

    tomatoes = build_tomatoes(detector, np.zeros((100, 100, 3), dtype=np.uint8), depth, INTRINSICS)

    assert tomatoes == []


def test_build_tomatoes_ids_are_sequential_despite_dropped_detections():
    detector = _FakeDetector([_det('unripe'), _det('ripe', u=20, v=20), _det('rotten', u=80, v=80)])
    depth = np.full((100, 100), 200.0)

    tomatoes = build_tomatoes(detector, np.zeros((100, 100, 3), dtype=np.uint8), depth, INTRINSICS)

    assert [t['tomato_id'] for t in tomatoes] == [0, 1]
    assert [t['grade'] for t in tomatoes] == ['NORMAL', 'DISCARD']
