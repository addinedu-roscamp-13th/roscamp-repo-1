#!/usr/bin/env python3
"""RP-110  TomatoDetector.detect_instances() — bbox -> 픽셀 원 마스크(padding) 검증.

실제 YOLO 모델 없이도 돌 수 있도록 model 속성만 가진 최소 스탠드인으로
detect_instances()의 순수 로직(중심/반지름 계산, 미지원 클래스 skip)만
검증한다.

실행:
  PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pytest src/dg_ai_service/test/test_detect_instances.py -v
"""
import numpy as np

from dg_ai_service.yolo_detector import TomatoDetector


class _FakeBox:
    def __init__(self, cls_index, confidence, xyxy):
        self.cls = [cls_index]
        self.conf = [confidence]
        self.xyxy = [np.array(xyxy, dtype=float)]


class _FakeResult:
    def __init__(self, boxes):
        self.boxes = boxes


class _FakeModel:
    names = {0: 'ripe', 1: 'unripe', 2: 'rotten', 3: 'disease'}

    def __init__(self, boxes):
        self._boxes = boxes

    def predict(self, img, conf, verbose):
        return [_FakeResult(self._boxes)]


class _DetectorStub:
    """TomatoDetector.__init__(실제 모델 파일 필요)을 거치지 않는 테스트용 스탠드인."""

    detect_instances = TomatoDetector.detect_instances

    def __init__(self, model, conf=0.4):
        self.model = model
        self.conf = conf


def test_detect_instances_derives_circle_from_bbox_with_padding():
    box = _FakeBox(0, 0.9, [10, 20, 30, 60])  # w=20, h=40 -> min/2=10
    detector = _DetectorStub(_FakeModel([box]))

    detections = detector.detect_instances(np.zeros((100, 100, 3), dtype=np.uint8), padding_px=5.0)

    assert len(detections) == 1
    det = detections[0]
    assert det['label'] == 'ripe'
    assert det['confidence'] == 0.9
    assert det['center_u'] == 20  # (10+30)/2
    assert det['center_v'] == 40  # (20+60)/2
    assert det['radius_px'] == 15  # 20/2 + 5


def test_detect_instances_skips_unknown_class():
    box = _FakeBox(99, 0.5, [0, 0, 10, 10])
    detector = _DetectorStub(_FakeModel([box]))

    detections = detector.detect_instances(np.zeros((10, 10, 3), dtype=np.uint8))

    assert detections == []


def test_detect_instances_no_boxes():
    class _NoBoxModel:
        names = {}

        def predict(self, img, conf, verbose):
            return [_FakeResult(None)]

    detector = _DetectorStub(_NoBoxModel())

    assert detector.detect_instances(np.zeros((10, 10, 3), dtype=np.uint8)) == []


def test_detect_instances_multiple_boxes_get_sequential_order():
    boxes = [
        _FakeBox(2, 0.8, [0, 0, 10, 10]),   # rotten
        _FakeBox(3, 0.7, [20, 20, 30, 30]),  # disease
    ]
    detector = _DetectorStub(_FakeModel(boxes))

    detections = detector.detect_instances(np.zeros((40, 40, 3), dtype=np.uint8))

    assert [d['label'] for d in detections] == ['rotten', 'disease']
