#!/usr/bin/env python3
"""RP-114  E0 ③④ Fleet 텔레메트리 취합 노드 테스트 (ROS 노드 배선 검증).

취합 로직 자체는 test_fleet_collector.py 가 ROS 없이 검증한다. 여기서는 '노드가 제대로
배선됐는지' — 로봇별 토픽을 실제로 구독하고, 자체 타이머로 QT 토픽에 내보내는지 — 를 본다.

  · 가짜 DG 가 /dg_01/telemetry, /dg_02/telemetry 로 RobotTelemetry 를 발행
  · 취합 노드가 /automato/dashboard/fleet_telemetry 로 robots[] 를 만들어 발행
  · [삭제 예정] 옛 /automato/telemetry/fleet 로 들어오는 길도 아직 살아 있는지

실행 (TESTING.md 규약):
  source /opt/ros/jazzy/setup.bash
  source <automato_interfaces install>/setup.bash
  PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pytest test/test_fleet_aggregator.py -v
"""
import os
import sys
import threading
import time

from automato_interfaces.msg import (
    DdagiTelemetry,
    DdagoTelemetry,
    FleetTelemetry,
    RobotTelemetry,
    ServoStatus,
)
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from automato_control_service.fleet_telemetry_aggregator import (  # noqa: E402
    FleetTelemetryAggregator,
)

LEGACY_TOPIC = '/automato/telemetry/fleet'
OUT_TOPIC = '/automato/dashboard/fleet_telemetry'
ROBOT_IDS = ['dg_01', 'dg_02']


def _wait_until(predicate, timeout=8.0):
    """조건이 참이 될 때까지 폴링(백그라운드 executor 가 콜백을 돌림)."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        if predicate():
            return True
        time.sleep(0.02)
    return False


def _make_ddago(task_id, x, y, battery, nav_status):
    """로봇이 보내는 형태 — robot_id 를 채우지 않는다(네임스페이스가 대신한다)."""
    m = DdagoTelemetry()
    m.header.frame_id = 'map'
    m.header.stamp.sec = 100
    m.task_id = task_id
    m.nav_status = nav_status
    m.is_charging = False
    m.x, m.y, m.yaw = x, y, 1.57
    m.battery_percent = battery
    m.battery_voltage = 12.1
    m.us_range_m = 0.42
    return m


def _make_ddagi():
    m = DdagiTelemetry()
    m.header.frame_id = 'base_link'
    m.header.stamp.sec = 100
    m.task_id = 1024
    m.is_paused = False
    m.joint_angles = [10.2, -30.5, 45.0, 0.0, -12.3, 5.5]
    m.tcp_coords = [160.0, 30.0, 200.0, 0.0, 0.0, 0.0]
    servos = []
    for j in range(7):
        s = ServoStatus()
        s.joint_no = j + 1
        s.voltage_ok = True
        s.temperature = 40 - j
        s.current = 0.2 + j * 0.01
        s.overload = False
        s.gripper_value = 85 if j == 6 else 0   # 7번(그리퍼)만 값 있음
        servos.append(s)
    m.servo_health = servos
    return m


@pytest.fixture
def ctx():
    """취합 노드 + 가짜 DG/QT 헬퍼를 백그라운드 스핀.

    publish_rate_hz 를 올려 테스트가 1초씩 기다리지 않게 한다(기본 1Hz 는 운영값).
    """
    rclpy.init()
    agg = FleetTelemetryAggregator(parameter_overrides=[
        Parameter('robot_ids', Parameter.Type.STRING_ARRAY, ROBOT_IDS),
        Parameter('publish_rate_hz', Parameter.Type.DOUBLE, 20.0),
    ])
    helper = rclpy.create_node('test_fleet_aggregator_helper')

    executor = SingleThreadedExecutor()
    executor.add_node(agg)
    executor.add_node(helper)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()

    yield agg, helper

    executor.shutdown()
    agg.destroy_node()
    helper.destroy_node()
    rclpy.shutdown()


def test_aggregates_per_robot_topics(ctx):
    """로봇별 토픽 2개를 구독해 robots[] 한 배열로 묶어 발행한다."""
    _agg, helper = ctx
    received = []
    helper.create_subscription(
        FleetTelemetry, OUT_TOPIC, lambda m: received.append(m), 10)

    pub1 = helper.create_publisher(RobotTelemetry, '/dg_01/telemetry', 10)
    pub2 = helper.create_publisher(RobotTelemetry, '/dg_02/telemetry', 10)

    src1 = RobotTelemetry()
    src1.ddagos = [_make_ddago(1024, 3.21, 1.05, 78.5, 'NAVIGATING')]
    src1.ddagis = [_make_ddagi()]
    src2 = RobotTelemetry()
    src2.ddagos = [_make_ddago(0, 5.10, 2.30, 62.0, 'IDLE')]   # 팔 없는 세트

    def both_arrived():
        # 연결이 성립할 때까지 매 폴링마다 재발행(테스트 관례).
        pub1.publish(src1)
        pub2.publish(src2)
        return any(len(m.robots) == 2 for m in received)

    assert _wait_until(both_arrived), '취합 출력에서 로봇 2대를 못 받음'
    out = next(m for m in reversed(received) if len(m.robots) == 2)

    # --- 취합 결과: robot_id 는 ACS 가 구독 네임스페이스에서 채운다 ---
    assert [m.robot_id for m in out.robots] == ['dg_01', 'dg_02']

    # --- 원본 손실 없음(QT 는 진단 목적이라 축약하지 않는다) ---
    dg01 = out.robots[0].telemetry
    assert dg01.ddagos[0].task_id == 1024
    assert dg01.ddagos[0].nav_status == 'NAVIGATING'
    assert dg01.ddagos[0].x == pytest.approx(3.21, abs=1e-3)
    assert dg01.ddagos[0].battery_percent == pytest.approx(78.5, abs=1e-2)
    assert dg01.ddagos[0].header.frame_id == 'map'
    assert dg01.ddagos[0].header.stamp.sec == 100, '로봇 stamp 가 덮어써졌다'

    ddagi = dg01.ddagis[0]
    assert list(ddagi.joint_angles) == pytest.approx(
        [10.2, -30.5, 45.0, 0.0, -12.3, 5.5], abs=1e-3)
    assert len(ddagi.servo_health) == 7
    assert ddagi.servo_health[6].gripper_value == 85   # 그리퍼 값 보존
    assert ddagi.servo_health[6].joint_no == 7

    # --- 팔 없는 세트는 ddagis 가 빈 배열(정상) ---
    assert len(out.robots[1].telemetry.ddagis) == 0

    # --- [삭제 예정] 옛 필드도 함께 채워 QT 무수정 동작을 유지 ---
    assert [d.robot_id for d in out.ddagos] == ['dg_01', 'dg_02']
    assert [a.robot_id for a in out.ddagis] == ['dg_01']


def test_legacy_fleet_input_still_works(ctx):
    """[삭제 예정] 팀원의 DG 이전 전까지 옛 경로로 들어오는 길도 살아 있어야 한다."""
    _agg, helper = ctx
    received = []
    helper.create_subscription(
        FleetTelemetry, OUT_TOPIC, lambda m: received.append(m), 10)
    pub = helper.create_publisher(FleetTelemetry, LEGACY_TOPIC, 10)

    # 옛 구조엔 네임스페이스가 없어 payload robot_id 로만 로봇을 가른다.
    legacy = FleetTelemetry()
    d = _make_ddago(7, 1.0, 2.0, 55.0, 'IDLE')
    d.robot_id = 'dg_09'          # robot_ids 파라미터에 없는 로봇 → 옛 경로로만 들어온다
    legacy.ddagos = [d]

    def arrived():
        pub.publish(legacy)
        return any(any(m.robot_id == 'dg_09' for m in msg.robots)
                   for msg in received)

    assert _wait_until(arrived), '옛 fleet 경로로 들어온 로봇이 취합되지 않음'
