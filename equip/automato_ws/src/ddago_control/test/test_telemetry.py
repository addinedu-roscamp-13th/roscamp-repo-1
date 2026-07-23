#!/usr/bin/env python3
"""RP-75  E0 텔레메트리 Publisher 단위 테스트.

가짜 소스 토픽(odom/amcl_pose/battery(percent·voltage)/navigate status)을
발행해 TelemetryPublisher 가 telemetry 로 취합·발행하는지 검증한다.
(초음파 us_range_m 은 미사용 — 항상 0.0 발행을 검증한다. 노드 독스트링 참고.)
로봇 없이(가짜 pub) 로직만 확인하는 "1단계 검증"의 자동화판이다.

TESTING.md 규약: SingleThreadedExecutor 로 노드+헬퍼를 한 스레드에서 스핀,
future/조건은 폴링으로 대기.

실행:
  source /opt/ros/jazzy/setup.bash
  cd equip/automato_ws && source install/setup.bash
  PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pytest src/ddago_control/test/test_telemetry.py -v
"""
import math
import threading
import time

from action_msgs.msg import GoalStatus, GoalStatusArray
from automato_interfaces.msg import DdagoTelemetry
from ddago_control.telemetry_publisher import TelemetryPublisher
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32, Int64


def _wait_until(predicate, timeout=5.0):
    """조건이 참이 될 때까지 폴링(백그라운드 executor 가 콜백을 돌림)."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        if predicate():
            return True
        time.sleep(0.02)
    return False


@pytest.fixture
def ctx():
    """가짜 소스/수집용 헬퍼와 TelemetryPublisher 를 백그라운드 스핀."""
    rclpy.init()
    # 발행 주기를 20Hz 로 올려 테스트를 빠르게 한다(실제 1Hz 는 실물 검증에서 확인).
    node = TelemetryPublisher(parameter_overrides=[
        Parameter('publish_rate_hz', Parameter.Type.DOUBLE, 20.0),
    ])
    helper = rclpy.create_node('test_telemetry_helper')

    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(helper)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()

    yield node, helper

    executor.shutdown()
    node.destroy_node()
    helper.destroy_node()
    rclpy.shutdown()


def _make_source_pubs(helper):
    """노드가 구독하는 소스 토픽들에 대응하는 가짜 발행자 생성."""
    # 액션 status 와 current_task 는 노드가 TRANSIENT_LOCAL 로 구독하므로
    # 발행도 같은 프로파일로 맞춰야 매칭된다(QoS 가 어긋나면 연결 자체가 안 된다).
    latched_qos = QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    return {
        'odom': helper.create_publisher(Odometry, 'odom', 10),
        'amcl': helper.create_publisher(
            PoseWithCovarianceStamped, 'amcl_pose', 10),
        'batt_pct': helper.create_publisher(Float32, 'battery/percent', 10),
        'batt_volt': helper.create_publisher(Float32, 'battery/voltage', 10),
        'status': helper.create_publisher(
            GoalStatusArray, 'navigate_to_pose/_action/status', latched_qos),
        'task': helper.create_publisher(
            Int64, '/ddago/current_task', latched_qos),
    }


def _publish_nav_status(pubs, status, sec):
    """Nav2 액션 status 를 한 건짜리 GoalStatusArray 로 발행."""
    st = GoalStatusArray()
    gs = GoalStatus()
    gs.status = status
    # 노드는 status_list 중 stamp 가 가장 최신인 것을 고른다 → sec 로 순서를 준다.
    gs.goal_info.stamp.sec = sec
    st.status_list = [gs]
    pubs['status'].publish(st)


def _subscribe_telemetry(helper, sink):
    # 노드가 절대명 /ddago/telemetry 로 발행하므로 구독도 같은 이름으로 맞춘다.
    helper.create_subscription(
        DdagoTelemetry, '/ddago/telemetry', lambda m: sink.append(m), 10)


def _publish_all_sources(pubs, amcl_yaw):
    """모든 소스에 알아볼 수 있는 가짜값을 한 번씩 발행."""
    # odom 은 amcl 과 다른 값 → amcl 이 우선 선택되는지 확인용.
    odom = Odometry()
    odom.pose.pose.position.x = 9.0
    odom.pose.pose.position.y = 9.0
    odom.pose.pose.orientation.w = 1.0
    pubs['odom'].publish(odom)

    amcl = PoseWithCovarianceStamped()
    amcl.pose.pose.position.x = 1.0
    amcl.pose.pose.position.y = 2.0
    amcl.pose.pose.orientation.z = math.sin(amcl_yaw / 2.0)
    amcl.pose.pose.orientation.w = math.cos(amcl_yaw / 2.0)
    pubs['amcl'].publish(amcl)

    pct = Float32()
    pct.data = 78.0
    pubs['batt_pct'].publish(pct)

    volt = Float32()
    volt.data = 12.3
    pubs['batt_volt'].publish(volt)

    st = GoalStatusArray()
    gs = GoalStatus()
    gs.status = GoalStatus.STATUS_EXECUTING
    gs.goal_info.stamp.sec = 100
    st.status_list = [gs]
    pubs['status'].publish(st)


def test_publishes_continuously(ctx):
    """소스가 하나도 없어도 텔레메트리가 계속 발행된다(로봇 생존 신호)."""
    _node, helper = ctx
    received = []
    _subscribe_telemetry(helper, received)
    assert _wait_until(lambda: len(received) >= 3, timeout=5.0), \
        '텔레메트리가 주기적으로 발행되지 않음'


def test_defaults_before_sources(ctx):
    """소스 수신 전에는 필드가 기본값(0/IDLE)으로 안전하게 발행된다."""
    _node, helper = ctx
    received = []
    _subscribe_telemetry(helper, received)
    assert _wait_until(lambda: len(received) >= 1, timeout=5.0)
    msg = received[0]
    # ddago 는 robot_id 를 채우지 않는다(빈 문자열) — 로봇 식별은 dcs 몫.
    assert msg.robot_id == ''
    assert msg.nav_status == 'IDLE'
    assert msg.task_id == 0          # goal 을 한 번도 받지 않았으면 0
    assert msg.x == pytest.approx(0.0)
    assert msg.y == pytest.approx(0.0)
    assert not msg.is_charging


def test_fields_reflect_sources(ctx):
    """가짜 소스값이 텔레메트리 각 필드에 실제로 반영된다(하드코딩 아님)."""
    _node, helper = ctx
    pubs = _make_source_pubs(helper)
    received = []
    _subscribe_telemetry(helper, received)

    amcl_yaw = math.pi / 2  # 90도

    def reflected():
        _publish_all_sources(pubs, amcl_yaw)
        # 배터리 전압(12.3)을 반영 완료 신호로 쓴다 — 기본값 0.0 과 뚜렷이 구분되는 값.
        return bool(received) and \
            received[-1].battery_voltage == pytest.approx(12.3, abs=1e-3)

    assert _wait_until(reflected, timeout=8.0), \
        '텔레메트리에 소스값이 반영되지 않음'

    msg = received[-1]
    assert msg.robot_id == ''   # ddago 는 robot_id 를 안 채운다(dcs 가 채움)
    assert msg.x == pytest.approx(1.0)             # amcl 우선
    assert msg.y == pytest.approx(2.0)
    assert msg.yaw == pytest.approx(amcl_yaw, abs=1e-3)
    assert msg.battery_voltage == pytest.approx(12.3, abs=1e-3)
    assert msg.battery_percent == pytest.approx(78.0, abs=1e-2)  # battery/percent
    assert not msg.is_charging   # 핑키는 충전상태 미제공 → 항상 False
    # 초음파 미사용 — 어떤 소스를 발행해도 us_range_m 은 항상 0.0 이어야 한다.
    assert msg.us_range_m == pytest.approx(0.0)
    assert msg.nav_status == 'NAVIGATING'          # EXECUTING → NAVIGATING


def test_odom_fallback_when_no_amcl(ctx):
    """위치: amcl 이 없으면 odom 좌표로 fallback 한다."""
    _node, helper = ctx
    pubs = _make_source_pubs(helper)
    received = []
    _subscribe_telemetry(helper, received)

    def odom_only():
        odom = Odometry()
        odom.pose.pose.position.x = 5.5
        odom.pose.pose.position.y = 6.6
        odom.pose.pose.orientation.w = 1.0
        pubs['odom'].publish(odom)
        return bool(received) and received[-1].x == pytest.approx(5.5, abs=1e-3)

    assert _wait_until(odom_only, timeout=8.0), 'odom fallback 이 반영되지 않음'
    msg = received[-1]
    assert msg.x == pytest.approx(5.5)
    assert msg.y == pytest.approx(6.6)
    assert msg.header.frame_id == 'odom'


def test_task_id_from_current_task(ctx):
    """/ddago/current_task 로 받은 task_id 가 텔레메트리에 실린다."""
    _node, helper = ctx
    pubs = _make_source_pubs(helper)
    received = []
    _subscribe_telemetry(helper, received)

    def task_reflected():
        t = Int64()
        t.data = 1024
        pubs['task'].publish(t)
        return bool(received) and received[-1].task_id == 1024

    assert _wait_until(task_reflected, timeout=8.0), \
        'current_task 가 텔레메트리 task_id 에 반영되지 않음'


def test_task_id_persists_after_goal_ends(ctx):
    """Goal 이 끝나도 task_id 는 0 으로 되돌아가지 않는다.

    ACS 는 한 task 를 예약 구간 단위로 쪼개 여러 goal 로 하달한다(문서 E2 4단계).
    goal 사이의 틈마다 0 이 되면 QT 화면에서 task_id 가 깜빡이고, 복귀 주행
    (22-1·E4, 같은 task_id 재사용) 추적도 끊긴다.
    """
    _node, helper = ctx
    pubs = _make_source_pubs(helper)
    received = []
    _subscribe_telemetry(helper, received)

    # goal 하달: task_id 가 실리고 주행이 시작된 상태를 만든다.
    def task_set():
        t = Int64()
        t.data = 1024
        pubs['task'].publish(t)
        _publish_nav_status(pubs, GoalStatus.STATUS_EXECUTING, 100)
        return bool(received) and received[-1].task_id == 1024

    assert _wait_until(task_set, timeout=8.0), 'task_id 가 실리지 않음'

    # goal 종료(SUCCEEDED). 다음 구간이 하달되기 전 틈에 해당한다.
    def goal_finished():
        _publish_nav_status(pubs, GoalStatus.STATUS_SUCCEEDED, 200)
        return received[-1].nav_status == 'IDLE'

    assert _wait_until(goal_finished, timeout=8.0), 'goal 종료가 반영되지 않음'
    assert received[-1].task_id == 1024, \
        'goal 종료 후 task_id 가 0 으로 되돌아갔다 (구간 사이 깜빡임 발생)'


def test_nav_status_has_only_two_values(ctx):
    """nav_status 는 IDLE / NAVIGATING 두 값뿐이다.

    특히 ABORTED(주행 실패)가 'FAILED' 로 남으면, 그 값이 다음 goal 까지 latch 되어
    E1 가용 조건(nav_status = IDLE)을 영영 통과하지 못하는 교착이 생긴다.
    """
    _node, helper = ctx
    pubs = _make_source_pubs(helper)
    received = []
    _subscribe_telemetry(helper, received)

    # 먼저 주행 중으로 만든다. (초기값이 IDLE 이라 곧바로 확인하면 가짜 통과가 된다)
    def navigating():
        _publish_nav_status(pubs, GoalStatus.STATUS_EXECUTING, 100)
        return bool(received) and received[-1].nav_status == 'NAVIGATING'

    assert _wait_until(navigating, timeout=8.0), '주행 중 상태가 반영되지 않음'

    # 취소 진행 중은 아직 감속 중이므로 NAVIGATING 을 유지한다.
    def canceling():
        _publish_nav_status(pubs, GoalStatus.STATUS_CANCELING, 200)
        return received[-1].nav_status == 'NAVIGATING'

    assert _wait_until(canceling, timeout=8.0), \
        'CANCELING 이 NAVIGATING 으로 유지되지 않음'

    # 주행 실패는 IDLE 로 돌아와야 한다 (실패 사실은 Navigate Result 가 전달).
    def aborted_is_idle():
        _publish_nav_status(pubs, GoalStatus.STATUS_ABORTED, 300)
        return received[-1].nav_status == 'IDLE'

    assert _wait_until(aborted_is_idle, timeout=8.0), \
        'ABORTED 가 IDLE 로 매핑되지 않음 (로봇이 배정 교착에 빠진다)'
