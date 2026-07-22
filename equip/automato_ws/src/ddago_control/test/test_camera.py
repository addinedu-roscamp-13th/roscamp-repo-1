#!/usr/bin/env python3
"""RP-76  E2: 카메라 노드(CaptureFrame 서버) 단위 테스트 — file 모드.

웹캠 없이(source='file') CameraNode 가
  요청 수신 → 정지 이미지 1장을 sensor_msgs/Image 로 응답
을 계약대로 하는지, 그리고 이미지가 없을 때 예외로 죽지 않고 success=false 로
알리는지 검증한다.

device 모드(cv2.VideoCapture)는 실물 웹캠이 있어야 해 여기서 다루지 않는다
(실물 검증은 rp76 필드 테스트에서).

실행:
  source /opt/ros/jazzy/setup.bash
  cd equip/automato_ws && source install/setup.bash
  PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pytest src/ddago_control/test/test_camera.py -v
"""
import tempfile
import threading
import time

import cv2
import numpy as np
from automato_interfaces.srv import CaptureFrame
from ddago_control.camera_node import CameraNode
import pytest
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter


def _wait_until(predicate, timeout=5.0):
    """조건이 참이 될 때까지 폴링(백그라운드 executor 가 콜백을 돌림)."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        if predicate():
            return True
        time.sleep(0.02)
    return False


class _Spun:
    """CameraNode + 요청용 헬퍼 노드를 MultiThreadedExecutor 로 백그라운드 스핀."""

    def __init__(self, source, image_path):
        self.node = CameraNode(parameter_overrides=[
            Parameter('source', Parameter.Type.STRING, source),
            Parameter('image_path', Parameter.Type.STRING, image_path),
        ])
        self.helper = rclpy.create_node('test_camera_helper')
        self.client = self.helper.create_client(
            CaptureFrame, '/ddago/capture_frame')

        self.executor = MultiThreadedExecutor()
        self.executor.add_node(self.node)
        self.executor.add_node(self.helper)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()

    def call(self, task_id=1, waypoint_id=1, timeout=5.0):
        assert self.client.wait_for_service(timeout_sec=timeout), \
            'CaptureFrame 서비스가 뜨지 않음'
        req = CaptureFrame.Request()
        req.task_id = task_id
        req.waypoint_id = waypoint_id
        fut = self.client.call_async(req)
        assert _wait_until(fut.done, timeout=timeout), 'CaptureFrame 응답 없음'
        return fut.result()

    def shutdown(self):
        self.executor.shutdown()
        self.node.destroy_node()
        self.helper.destroy_node()


@pytest.fixture
def tmp_image():
    """32x24 검은 이미지를 임시 JPEG 로 만들어 그 경로를 준다."""
    path = f'{tempfile.mkdtemp()}/frame.jpg'
    cv2.imwrite(path, np.zeros((24, 32, 3), dtype=np.uint8))
    return path


def test_file_mode_returns_frame(tmp_image):
    """file 모드: 지정 이미지를 bgr8 Image 로 응답한다."""
    rclpy.init()
    spun = _Spun('file', tmp_image)
    try:
        resp = spun.call(task_id=7, waypoint_id=10)
        assert resp.success is True
        assert resp.message == ''
        assert resp.image.width == 32
        assert resp.image.height == 24
        assert resp.image.encoding == 'bgr8'
        # bgr8 = 픽셀당 3바이트 → 잘림 없이 통째로 실렸는지 확인
        assert len(resp.image.data) == 32 * 24 * 3
    finally:
        spun.shutdown()
        rclpy.shutdown()


def test_file_mode_missing_image_reports_failure():
    """이미지 경로가 잘못되면 예외로 죽지 않고 success=false + 사유를 준다."""
    rclpy.init()
    spun = _Spun('file', '/nonexistent/nope.jpg')
    try:
        resp = spun.call()
        assert resp.success is False
        assert resp.message != ''
    finally:
        spun.shutdown()
        rclpy.shutdown()
