#!/usr/bin/env python3
"""RP-76  E2: 카메라 노드(CaptureFrame 서버) 단위 테스트 — file 모드.

웹캠 없이(source='file') CameraNode 가
  요청 수신 → 정지 이미지 1장을 sensor_msgs/Image 로 응답
을 계약대로 하는지, 그리고 이미지가 없을 때 예외로 죽지 않고 success=false 로
알리는지 검증한다.

device 모드의 실제 촬영 경로(cv2.VideoCapture)는 실물 웹캠이 있어야 해 여기서
다루지 않는다(실물 검증은 rp76 필드 테스트에서). 다만 유휴 release(배터리 절전)와
AE 워밍업 로직은 가짜 cap(_FakeCap)을 주입해 하드웨어 없이 검증한다.

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


# ---------------------------------------------------------------------- #
# device 모드 유휴 release / AE 워밍업 — 실물 웹캠 없이 로직만 검증
# ---------------------------------------------------------------------- #
class _FakeCap:
    """cv2.VideoCapture 대역: release/grab 호출을 기록만 한다."""

    def __init__(self):
        self.released = False
        self.grab_count = 0

    def isOpened(self):
        return not self.released

    def set(self, *_args):
        return True

    def grab(self):
        self.grab_count += 1
        return True

    def release(self):
        self.released = True


def _device_node(**params):
    """device 모드 CameraNode. device_index=99(없는 장치) 강제라 어느 PC 에서
    돌려도 실물 웹캠을 건드리지 않고 '열기 실패' 경로로 즉시 빠진다."""
    overrides = [
        Parameter('source', Parameter.Type.STRING, 'device'),
        Parameter('device_index', Parameter.Type.INTEGER, 99),
    ]
    for name, val in params.items():
        ptype = (Parameter.Type.DOUBLE if isinstance(val, float)
                 else Parameter.Type.INTEGER)
        overrides.append(Parameter(name, ptype, val))
    return CameraNode(parameter_overrides=overrides)


def test_idle_release_after_timeout():
    """마지막 사용 후 idle_release_sec 지나면 타이머 콜백이 cap 을 놓아 준다."""
    rclpy.init()
    node = _device_node(idle_release_sec=10.0)
    try:
        fake = _FakeCap()
        node._cap = fake
        node._last_use = time.monotonic() - 11.0   # 유휴 11초 경과로 위장
        node._on_idle_check()
        assert fake.released is True
        assert node._cap is None                    # 다음 요청 때 재오픈되도록 비움
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_idle_within_window_keeps_open():
    """유휴 시간이 문턱 미만이면 cap 을 유지한다(연속 촬영은 빨라야 한다)."""
    rclpy.init()
    node = _device_node(idle_release_sec=10.0)
    try:
        fake = _FakeCap()
        node._cap = fake
        node._last_use = time.monotonic()           # 방금 사용
        node._on_idle_check()
        assert fake.released is False
        assert node._cap is fake
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_idle_timer_disabled_when_nonpositive():
    """idle_release_sec<=0 이면 타이머를 안 만든다(예전 '상시 열림' 동작 유지)."""
    rclpy.init()
    node = _device_node(idle_release_sec=0.0)
    try:
        assert node._idle_timer is None
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_warmup_grabs_on_fresh_open():
    """새로 연 경우에만 AE 워밍업 grab 이 warmup_frames 만큼 나간다."""
    rclpy.init()
    node = _device_node(warmup_frames=15)
    try:
        fake = _FakeCap()
        node._open_webcam = lambda: (fake, 'fake')  # 실물 오픈을 대역으로 치환
        assert node._ensure_device() is True
        assert fake.grab_count == 15                # 새로 열었으니 워밍업 발생
        assert node._cap is fake

        fake.grab_count = 0
        assert node._ensure_device() is True        # 이미 열려 있으면
        assert fake.grab_count == 0                 # 워밍업을 반복하지 않는다
    finally:
        node.destroy_node()
        rclpy.shutdown()
