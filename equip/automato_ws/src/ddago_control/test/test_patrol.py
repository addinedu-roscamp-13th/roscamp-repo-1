#!/usr/bin/env python3
"""RP-76  E2 Patrol Action 서버 단위 테스트.

로봇 없이(가짜 Nav2 서버·가짜 AnalyzeFrame 서비스·가짜 카메라) PatrolServer 가
  주행 → 도착 → 프레임 grab → analyze_frame 호출 → result_code 반환
흐름을 올바르게 수행하는지 검증하는 "1단계 검증"의 자동화판이다.

PatrolServer 는 execute 콜백이 Nav2 결과를 기다리며 블로킹하므로(그 사이 다른
콜백이 돌아야 함) SingleThreadedExecutor 로는 데드락이 난다. 따라서 여기서는
MultiThreadedExecutor 로 노드+헬퍼를 백그라운드 스핀하고, future/조건은 폴링한다.

실행:
  source /opt/ros/jazzy/setup.bash
  cd equip/automato_ws && source install/setup.bash
  PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pytest src/ddago_control/test/test_patrol.py -v
"""
import threading
import time

from automato_interfaces.action import Patrol
from automato_interfaces.srv import AnalyzeFrame
from ddago_control.patrol_server import PatrolServer
from nav2_msgs.action import NavigateToPose
import pytest
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


def _wait_until(predicate, timeout=5.0):
    """조건이 참이 될 때까지 폴링(백그라운드 executor 가 콜백을 돌림)."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        if predicate():
            return True
        time.sleep(0.02)
    return False


class _Harness:
    """PatrolServer 를 둘러싸는 가짜 환경(테스트 스탠드인)을 한 곳에 모은다."""

    def __init__(self):
        # nav_mode 로 가짜 Nav2 서버 동작을 바꾼다: success/cancelable/abort.
        self.nav_mode = 'success'
        self.analyze_requests = []   # 가짜 HQ 가 수신한 AnalyzeFrame.Request 들
        self.feedbacks = []          # Patrol 액션 client 가 받은 Feedback 들
        self.cbg = ReentrantCallbackGroup()

        self.node = PatrolServer(parameter_overrides=[
            Parameter('arrival_settle_sec', Parameter.Type.DOUBLE, 0.05),
            Parameter('nav2_wait_sec', Parameter.Type.DOUBLE, 5.0),
            Parameter('nav2_result_timeout_sec', Parameter.Type.DOUBLE, 15.0),
        ])
        self.helper = rclpy.create_node('test_patrol_helper')

        # 가짜 Nav2 NavigateToPose 액션 서버
        self._nav_server = ActionServer(
            self.helper, NavigateToPose, 'navigate_to_pose',
            execute_callback=self._nav_execute,
            cancel_callback=lambda _gh: CancelResponse.ACCEPT,
            callback_group=self.cbg)

        # 가짜 HQ AnalyzeFrame 서비스(요청 기록 후 수락)
        self.helper.create_service(
            AnalyzeFrame, '/dg/analyze_frame', self._analyze_cb,
            callback_group=self.cbg)

        # 가짜 카메라 이미지 발행자
        self.cam_pub = self.helper.create_publisher(
            Image, 'image_raw', qos_profile_sensor_data)

        # Patrol 액션 client (노드의 서버 ddago/patrol 호출)
        self.patrol_client = ActionClient(
            self.helper, Patrol, 'ddago/patrol', callback_group=self.cbg)

    # ------- 가짜 Nav2 서버 동작 ------- #
    def _nav_execute(self, goal_handle):
        # 진행 중 피드백 1회 발행(PatrolServer 가 Patrol 피드백으로 중계하는지 확인용)
        fb = NavigateToPose.Feedback()
        fb.current_pose.header.frame_id = 'map'
        fb.current_pose.pose.position.x = 0.5
        fb.current_pose.pose.position.y = 0.25
        fb.current_pose.pose.orientation.w = 1.0
        goal_handle.publish_feedback(fb)

        if self.nav_mode == 'abort':
            goal_handle.abort()
            return NavigateToPose.Result()
        if self.nav_mode == 'cancelable':
            # 취소 요청이 올 때까지 주행 중인 척 대기.
            for _ in range(400):
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    return NavigateToPose.Result()
                time.sleep(0.05)
            goal_handle.succeed()
            return NavigateToPose.Result()
        # success: 피드백 전달 여유를 준 뒤 도착 처리
        time.sleep(0.1)
        goal_handle.succeed()
        return NavigateToPose.Result()

    def _analyze_cb(self, request, response):
        self.analyze_requests.append(request)
        response.accepted = True
        response.request_id = 'req-test-1'
        return response

    # ------- 카메라 프레임 준비 ------- #
    def publish_camera(self, frame_id='pinky_cam', width=640, height=480):
        img = Image()
        img.header.frame_id = frame_id
        img.width = width
        img.height = height
        img.encoding = 'rgb8'
        # 최신 프레임이 노드에 확실히 캐시될 때까지 여러 번 발행.
        for _ in range(5):
            self.cam_pub.publish(img)
            time.sleep(0.03)
        _wait_until(lambda: self.node._latest_frame is not None, timeout=3.0)

    # ------- Patrol goal 전송 ------- #
    def send_patrol(self, task_id=1, wp_id=7, x=1.0, y=0.5):
        assert self.patrol_client.wait_for_server(timeout_sec=5.0), \
            'Patrol 서버가 뜨지 않음'
        goal = Patrol.Goal()
        goal.task_id = task_id
        goal.waypoint.waypoint_id = wp_id
        goal.waypoint.x = x
        goal.waypoint.y = y
        send_future = self.patrol_client.send_goal_async(
            goal, feedback_callback=lambda fb: self.feedbacks.append(fb.feedback))
        assert _wait_until(send_future.done, timeout=5.0), 'goal 전송 실패'
        gh = send_future.result()
        assert gh.accepted, 'goal 이 거부됨'
        return gh, gh.get_result_async()


@pytest.fixture
def harness():
    """가짜 환경 + PatrolServer 를 MultiThreadedExecutor 로 백그라운드 스핀."""
    rclpy.init()
    h = _Harness()

    executor = MultiThreadedExecutor()
    executor.add_node(h.node)
    executor.add_node(h.helper)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()

    yield h

    executor.shutdown()
    h.node.destroy_node()
    h.helper.destroy_node()
    rclpy.shutdown()


def test_arrive_captures_and_requests(harness):
    """도착(성공) → 프레임 grab → analyze_frame 호출, result_code=0."""
    harness.nav_mode = 'success'
    harness.publish_camera(frame_id='pinky_cam', width=640, height=480)

    _gh, result_future = harness.send_patrol(task_id=1, wp_id=7, x=1.0, y=0.5)
    assert _wait_until(result_future.done, timeout=10.0), '결과가 오지 않음'

    result = result_future.result().result
    assert result.result_code == 0
    assert result.message == 'arrived'

    # 주행 피드백이 Patrol 피드백으로 중계됐는지(waypoint_id 유지, Nav2 좌표 반영)
    assert _wait_until(lambda: len(harness.feedbacks) >= 1, timeout=2.0), \
        '피드백 미수신'
    assert any(fb.current_waypoint_id == 7 for fb in harness.feedbacks)
    assert any(abs(fb.current_x - 0.5) < 1e-3 for fb in harness.feedbacks)

    # 도착 후 grab 한 '그 이미지'로 analyze_frame 이 호출됐는지
    assert _wait_until(lambda: len(harness.analyze_requests) >= 1, timeout=3.0), \
        'analyze_frame 이 호출되지 않음'
    req = harness.analyze_requests[0]
    assert req.task_id == 1
    assert req.waypoint_id == 7
    assert req.image.width == 640
    assert req.image.height == 480
    assert req.image.header.frame_id == 'pinky_cam'


def test_missing_frame_still_succeeds(harness):
    """카메라 프레임이 없어도 주행은 성공(0). 분석요청만 스킵된다."""
    harness.nav_mode = 'success'
    # 카메라 미발행 → latest_frame 은 None

    _gh, result_future = harness.send_patrol(wp_id=3)
    assert _wait_until(result_future.done, timeout=10.0)

    assert result_future.result().result.result_code == 0
    # 프레임이 없으니 analyze_frame 은 호출되지 않아야 한다.
    time.sleep(0.5)
    assert len(harness.analyze_requests) == 0


def test_nav_abort_returns_code_1(harness):
    """Nav2 주행 실패(ABORTED) → result_code=1, 촬영/분석 안 함."""
    harness.nav_mode = 'abort'
    harness.publish_camera()

    _gh, result_future = harness.send_patrol(wp_id=5)
    assert _wait_until(result_future.done, timeout=10.0)

    assert result_future.result().result.result_code == 1
    time.sleep(0.5)
    assert len(harness.analyze_requests) == 0   # 도착 안 했으니 촬영 없음


def test_cancel_returns_code_2(harness):
    """주행 중 상위가 취소 → Nav2 goal 취소되고 result_code=2."""
    harness.nav_mode = 'cancelable'
    harness.publish_camera()

    gh, result_future = harness.send_patrol(wp_id=9)
    # 주행이 시작(피드백 수신)된 뒤 취소 요청.
    assert _wait_until(lambda: len(harness.feedbacks) >= 1, timeout=5.0), \
        '주행 피드백 미수신'
    gh.cancel_goal_async()

    assert _wait_until(result_future.done, timeout=10.0), '취소 결과 미수신'
    assert result_future.result().result.result_code == 2
    time.sleep(0.5)
    assert len(harness.analyze_requests) == 0   # 취소는 촬영 없음
