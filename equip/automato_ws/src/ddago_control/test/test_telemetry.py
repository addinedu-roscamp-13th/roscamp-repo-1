#!/usr/bin/env python3
"""RP-75  E0 텔레메트리 Publisher 단위 테스트.

가짜 소스 토픽(odom/amcl_pose/battery(percent·voltage)/us_sensor·range/navigate
status)을 발행해 TelemetryPublisher 가 telemetry 로 취합·발행하는지 검증한다.
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
from sensor_msgs.msg import Range
from std_msgs.msg import Float32


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
    # 액션 status 는 노드가 TRANSIENT_LOCAL 로 구독하므로 발행도 맞춰야 매칭된다.
    status_qos = QoSProfile(
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
        'range': helper.create_publisher(Range, 'us_sensor/range', 10),
        'status': helper.create_publisher(
            GoalStatusArray, 'navigate_to_pose/_action/status', status_qos),
    }


def _subscribe_telemetry(helper, sink):
    helper.create_subscription(
        DdagoTelemetry, 'telemetry', lambda m: sink.append(m), 10)


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

    rng = Range()
    rng.range = 0.4
    pubs['range'].publish(rng)

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
    assert msg.robot_id == 'dg_01'
    assert msg.nav_status == 'IDLE'
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
        return bool(received) and \
            received[-1].us_range_m == pytest.approx(0.4, abs=1e-3)

    assert _wait_until(reflected, timeout=8.0), \
        '텔레메트리에 소스값이 반영되지 않음'

    msg = received[-1]
    assert msg.robot_id == 'dg_01'
    assert msg.x == pytest.approx(1.0)             # amcl 우선
    assert msg.y == pytest.approx(2.0)
    assert msg.yaw == pytest.approx(amcl_yaw, abs=1e-3)
    assert msg.battery_voltage == pytest.approx(12.3, abs=1e-3)
    assert msg.battery_percent == pytest.approx(78.0, abs=1e-2)  # battery/percent
    assert not msg.is_charging   # 핑키는 충전상태 미제공 → 항상 False
    assert msg.us_range_m == pytest.approx(0.4, abs=1e-3)
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
