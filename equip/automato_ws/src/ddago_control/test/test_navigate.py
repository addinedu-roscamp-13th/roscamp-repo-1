#!/usr/bin/env python3
"""RP-113  E1/E2 Navigate Action 서버 단위 테스트.

로봇 없이(가짜 Nav2 주행·가짜 Nav2 Spin·가짜 AnalyzeFrame 서비스·가짜 카메라)
NavigateServer 가
  경로 배열 순차 주행 → 도착마다 보고 → capture 노드만 촬영 → result 반환
을 계약대로 수행하는지 검증한다.

NavigateServer 는 execute 콜백이 Nav2 결과를 기다리며 블로킹하므로(그 사이 다른
콜백이 돌아야 함) SingleThreadedExecutor 로는 데드락이 난다. 따라서 여기서는
MultiThreadedExecutor 로 노드+헬퍼를 백그라운드 스핀하고, future/조건은 폴링한다.

실행:
  source /opt/ros/jazzy/setup.bash
  cd equip/automato_ws && source install/setup.bash
  PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pytest src/ddago_control/test/test_navigate.py -v
"""
import math
import threading
import time

from automato_interfaces.action import Navigate
from automato_interfaces.msg import Waypoint
from automato_interfaces.srv import AnalyzeFrame
from ddago_control.navigate_server import NavigateServer
from nav2_msgs.action import NavigateToPose, Spin
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


def _wp(waypoint_id, x, y, yaw=0.0, capture=False):
    """테스트용 Waypoint 한 개를 짧게 만든다."""
    w = Waypoint()
    w.waypoint_id = int(waypoint_id)
    w.x = float(x)
    w.y = float(y)
    w.yaw = float(yaw)
    w.capture = bool(capture)
    return w


class _Harness:
    """NavigateServer 를 둘러싸는 가짜 환경(테스트 스탠드)을 한 곳에 모은다."""

    def __init__(self):
        # nav_mode 로 가짜 Nav2 주행 동작을 바꾼다: success / cancelable.
        self.nav_mode = 'success'
        # nav_fail_at 이 정수면 '그 순번의 주행 호출'에서 ABORTED 를 돌려준다(0-based).
        self.nav_fail_at = None

        self.nav_goals = []          # 가짜 Nav2 가 받은 NavigateToPose.Goal 들(순서대로)
        self.spin_goals = []         # 가짜 Nav2 가 받은 Spin.Goal 들
        self.analyze_requests = []   # 가짜 DCS 가 수신한 AnalyzeFrame.Request 들
        self.feedbacks = []          # Navigate 액션 client 가 받은 Feedback 들
        self.cbg = ReentrantCallbackGroup()

        self.node = NavigateServer(parameter_overrides=[
            Parameter('arrival_settle_sec', Parameter.Type.DOUBLE, 0.05),
            Parameter('nav2_wait_sec', Parameter.Type.DOUBLE, 5.0),
            Parameter('nav2_result_timeout_sec', Parameter.Type.DOUBLE, 15.0),
        ])
        self.helper = rclpy.create_node('test_navigate_helper')

        # 가짜 Nav2 NavigateToPose 액션 서버
        self._nav_server = ActionServer(
            self.helper, NavigateToPose, 'navigate_to_pose',
            execute_callback=self._nav_execute,
            cancel_callback=lambda _gh: CancelResponse.ACCEPT,
            callback_group=self.cbg)

        # 가짜 Nav2 Spin 액션 서버 (제자리 회전 — 짝 노드용)
        self._spin_server = ActionServer(
            self.helper, Spin, 'spin',
            execute_callback=self._spin_execute,
            cancel_callback=lambda _gh: CancelResponse.ACCEPT,
            callback_group=self.cbg)

        # 가짜 DCS AnalyzeFrame 서비스(요청 기록 후 수락)
        self.helper.create_service(
            AnalyzeFrame, '/dg/analyze_frame', self._analyze_cb,
            callback_group=self.cbg)

        # 가짜 카메라 이미지 발행자
        self.cam_pub = self.helper.create_publisher(
            Image, 'image_raw', qos_profile_sensor_data)

        # Navigate 액션 client (노드의 서버 /ddago/navigate 호출)
        self.nav_client = ActionClient(
            self.helper, Navigate, '/ddago/navigate', callback_group=self.cbg)

    # ------- 가짜 Nav2 주행 서버 ------- #
    def _nav_execute(self, goal_handle):
        idx = len(self.nav_goals)
        self.nav_goals.append(goal_handle.request)

        # 실제 Nav2 처럼 현재 위치 피드백을 준다. 목표 자세를 그대로 실어 보내
        # "도착했다"를 흉내내면, 서버의 _last_pose 캐시가 목표 yaw 로 갱신된다
        # (Spin 의 상대각 계산이 이 값을 기준으로 하므로 중요).
        fb = NavigateToPose.Feedback()
        fb.current_pose.header.frame_id = 'map'
        fb.current_pose.pose = goal_handle.request.pose.pose
        goal_handle.publish_feedback(fb)

        if self.nav_fail_at is not None and idx == self.nav_fail_at:
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

        time.sleep(0.05)
        goal_handle.succeed()
        return NavigateToPose.Result()

    # ------- 가짜 Nav2 Spin 서버 ------- #
    def _spin_execute(self, goal_handle):
        self.spin_goals.append(goal_handle.request)
        time.sleep(0.05)
        goal_handle.succeed()
        return Spin.Result()

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

    # ------- Navigate goal 전송 ------- #
    def send_navigate(self, waypoints, task_id=1):
        assert self.nav_client.wait_for_server(timeout_sec=5.0), \
            'Navigate 서버가 뜨지 않음'
        goal = Navigate.Goal()
        goal.task_id = task_id
        goal.waypoints = waypoints
        send_future = self.nav_client.send_goal_async(
            goal, feedback_callback=lambda fb: self.feedbacks.append(fb.feedback))
        assert _wait_until(send_future.done, timeout=5.0), 'goal 전송 실패'
        gh = send_future.result()
        assert gh.accepted, 'goal 이 거부됨'
        return gh, gh.get_result_async()


@pytest.fixture
def harness():
    """가짜 환경 + NavigateServer 를 MultiThreadedExecutor 로 백그라운드 스핀."""
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


# --------------------------------------------------------------------------- #
# 1) 배열 순차 주행
# --------------------------------------------------------------------------- #
def test_drives_every_waypoint_in_order(harness):
    """배열의 모든 waypoint 를 순서대로 주행하고 마지막 노드를 결과에 담는다."""
    wps = [_wp(3, 1.0, 0.0), _wp(4, 2.0, 0.0), _wp(10, 3.0, 0.0)]

    _gh, result_future = harness.send_navigate(wps)
    assert _wait_until(result_future.done, timeout=15.0), '결과가 오지 않음'

    result = result_future.result().result
    assert result.result_code == 0
    assert result.last_waypoint_id == 10          # 배열의 마지막 노드

    # Nav2 가 배열 순서대로, 개수만큼 호출됐는가
    assert len(harness.nav_goals) == 3
    xs = [g.pose.pose.position.x for g in harness.nav_goals]
    assert xs == pytest.approx([1.0, 2.0, 3.0])


def test_feedback_is_published_once_per_arrival(harness):
    """도착할 때마다 1회씩만 보고하고, waypoint_index 가 0,1,2 로 증가한다.

    주행 '중'에 보고하면 ACS 가 아직 지나지 않은 통로를 반납해버린다
    (patrol_dispatcher._passed_resources). 그래서 개수까지 검증한다.
    """
    wps = [_wp(3, 1.0, 0.0), _wp(4, 2.0, 0.0), _wp(10, 3.0, 0.0)]

    _gh, result_future = harness.send_navigate(wps)
    assert _wait_until(result_future.done, timeout=15.0)
    assert _wait_until(lambda: len(harness.feedbacks) >= 3, timeout=3.0), \
        '도착 보고 미수신'

    # 가짜 Nav2 가 주행 중 피드백을 1회씩 주지만, 그것이 그대로 중계되면 안 된다.
    time.sleep(0.3)
    assert len(harness.feedbacks) == 3, \
        f'도착 보고는 3회여야 하는데 {len(harness.feedbacks)}회 발행됨'

    assert [fb.waypoint_index for fb in harness.feedbacks] == [0, 1, 2]
    assert [fb.current_waypoint_id for fb in harness.feedbacks] == [3, 4, 10]


# --------------------------------------------------------------------------- #
# 2) last_waypoint_id — ACS 재계획의 기준점
# --------------------------------------------------------------------------- #
def test_midway_failure_reports_last_reached_node(harness):
    """중간에 주행이 실패하면 '실제로 도달한' 직전 노드를 담는다."""
    harness.nav_fail_at = 1        # 두 번째 주행(4번 노드)에서 ABORTED
    wps = [_wp(3, 1.0, 0.0), _wp(4, 2.0, 0.0), _wp(10, 3.0, 0.0)]

    _gh, result_future = harness.send_navigate(wps)
    assert _wait_until(result_future.done, timeout=15.0)

    result = result_future.result().result
    assert result.result_code == 1
    assert result.last_waypoint_id == 3     # 4번은 실패했으니 3번까지 간 것
    assert len(harness.nav_goals) == 2      # 실패 후 나머지는 시도하지 않는다


def test_failure_before_first_node_reports_minus_one(harness):
    """첫 노드에도 못 갔으면 -1. ACS 가 '세그먼트 진입 전'으로 해석해야 한다."""
    harness.nav_fail_at = 0        # 첫 주행부터 ABORTED
    wps = [_wp(3, 1.0, 0.0), _wp(4, 2.0, 0.0)]

    _gh, result_future = harness.send_navigate(wps)
    assert _wait_until(result_future.done, timeout=15.0)

    result = result_future.result().result
    assert result.result_code == 1
    assert result.last_waypoint_id == -1
    assert len(harness.feedbacks) == 0      # 도착한 곳이 없으니 보고도 없다


def test_empty_waypoints_is_rejected(harness):
    """빈 배열은 ACS 의 버그다. 즉시 실패로 알린다."""
    _gh, result_future = harness.send_navigate([])
    assert _wait_until(result_future.done, timeout=10.0)

    result = result_future.result().result
    assert result.result_code == 1
    assert result.last_waypoint_id == -1
    assert len(harness.nav_goals) == 0


# --------------------------------------------------------------------------- #
# 3) capture 플래그
# --------------------------------------------------------------------------- #
def test_capture_flag_decides_analyze_request(harness):
    """capture=true 인 노드에서만 분석을 요청한다(통과 노드는 촬영하지 않는다)."""
    harness.publish_camera()
    wps = [
        _wp(3, 1.0, 0.0, capture=False),    # 통과
        _wp(4, 2.0, 0.0, capture=False),    # 통과
        _wp(10, 3.0, 0.0, capture=True),    # 순찰 지점
    ]

    _gh, result_future = harness.send_navigate(wps, task_id=77)
    assert _wait_until(result_future.done, timeout=15.0)
    assert _wait_until(lambda: len(harness.analyze_requests) >= 1, timeout=3.0), \
        'analyze_frame 이 호출되지 않음'

    time.sleep(0.3)
    assert len(harness.analyze_requests) == 1, '촬영은 capture 노드에서만'
    req = harness.analyze_requests[0]
    assert req.task_id == 77
    assert req.waypoint_id == 10
    assert req.image.width == 640


def test_missing_frame_does_not_fail_navigation(harness):
    """카메라 프레임이 없어도 주행은 성공(0). 분석요청만 스킵된다.

    촬영 실패로 주행을 실패시키면 ACS 가 '통로 막힘'으로 오해해 우회·복귀를 밟는다.
    """
    # 카메라 미발행 → latest_frame 은 None
    wps = [_wp(10, 1.0, 0.0, capture=True)]

    _gh, result_future = harness.send_navigate(wps)
    assert _wait_until(result_future.done, timeout=15.0)

    assert result_future.result().result.result_code == 0
    time.sleep(0.3)
    assert len(harness.analyze_requests) == 0


# --------------------------------------------------------------------------- #
# 4) 짝 노드 — 제자리 회전 (E2 20-1)
# --------------------------------------------------------------------------- #
def test_same_position_spins_instead_of_driving(harness):
    """직전과 좌표가 같으면 주행이 아니라 제자리 회전으로 처리한다."""
    harness.publish_camera()
    wps = [
        _wp(10, 1.2, 3.4, yaw=1.57, capture=True),    # 주행 후 촬영
        _wp(18, 1.2, 3.4, yaw=-1.57, capture=True),   # 짝 — 제자리 회전 후 촬영
    ]

    _gh, result_future = harness.send_navigate(wps)
    assert _wait_until(result_future.done, timeout=15.0)

    result = result_future.result().result
    assert result.result_code == 0
    assert result.last_waypoint_id == 18

    assert len(harness.nav_goals) == 1, '같은 자리로 다시 주행하면 안 된다'
    assert len(harness.spin_goals) == 1, '짝 노드는 Spin 으로 처리해야 한다'
    # 양쪽 다 촬영 대상이므로 분석요청은 2회
    assert _wait_until(lambda: len(harness.analyze_requests) >= 2, timeout=3.0)
    assert [r.waypoint_id for r in harness.analyze_requests] == [10, 18]


def test_spin_target_yaw_is_relative_and_normalized(harness):
    """Spin.target_yaw 는 절대 방향이 아니라 (목표 - 현재)를 -pi~pi 로 접은 값이다."""
    wps = [
        _wp(10, 1.2, 3.4, yaw=1.57),
        _wp(18, 1.2, 3.4, yaw=-1.57),
    ]

    _gh, result_future = harness.send_navigate(wps)
    assert _wait_until(result_future.done, timeout=15.0)
    assert len(harness.spin_goals) == 1

    # 가짜 Nav2 가 목표 자세를 피드백으로 돌려주므로 현재 yaw 는 1.57 로 캐시된다.
    expected = math.atan2(math.sin(-1.57 - 1.57), math.cos(-1.57 - 1.57))
    assert harness.spin_goals[0].target_yaw == pytest.approx(expected, abs=1e-3)
    # 절대값(-1.57)을 그대로 실어 보내는 실수를 잡아낸다.
    assert abs(harness.spin_goals[0].target_yaw - (-1.57)) > 1.0


# --------------------------------------------------------------------------- #
# 5) 취소 (E2 22-1 실패 복귀)
# --------------------------------------------------------------------------- #
def test_cancel_returns_code_2(harness):
    """주행 중 상위가 취소하면 Nav2 goal 도 취소되고 result_code=2."""
    harness.nav_mode = 'cancelable'
    wps = [_wp(3, 1.0, 0.0), _wp(4, 2.0, 0.0)]

    gh, result_future = harness.send_navigate(wps)
    # 주행이 시작된 뒤 취소 요청.
    assert _wait_until(lambda: len(harness.nav_goals) >= 1, timeout=5.0), \
        '주행이 시작되지 않음'
    gh.cancel_goal_async()

    assert _wait_until(result_future.done, timeout=15.0), '취소 결과 미수신'
    assert result_future.result().result.result_code == 2
    assert len(harness.nav_goals) == 1      # 취소 후 다음 노드로 가지 않는다
