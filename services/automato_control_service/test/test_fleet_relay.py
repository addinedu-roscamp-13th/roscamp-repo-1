#!/usr/bin/env python3
"""RP-77  E0 ③④ Fleet 텔레메트리 릴레이 단위 테스트.

가짜 HQ가 /automato/telemetry/fleet 에 FleetTelemetry(ddago 2대 + ddagi 1대)를
발행하면, FleetTelemetryRelay 가 /automato/dashboard/fleet_telemetry 로
원본을 손실 없이 그대로 재발행하는지 검증한다(로봇 없이 로직만 확인).

실행 (TESTING.md 규약):
  source /opt/ros/jazzy/setup.bash
  source <automato_interfaces install>/setup.bash
  PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pytest test/test_fleet_relay.py -v
"""
import os
import sys
import threading
import time

from automato_interfaces.msg import (
    DdagiTelemetry,
    DdagoTelemetry,
    FleetTelemetry,
    ServoStatus,
)
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from automato_control_service.fleet_telemetry_relay import FleetTelemetryRelay

IN_TOPIC = '/automato/telemetry/fleet'
OUT_TOPIC = '/automato/dashboard/fleet_telemetry'


def _wait_until(predicate, timeout=8.0):
    """조건이 참이 될 때까지 폴링(백그라운드 executor 가 콜백을 돌림)."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        if predicate():
            return True
        time.sleep(0.02)
    return False


def _make_ddago(robot_id, task_id, x, y, battery, nav_status):
    m = DdagoTelemetry()
    m.header.frame_id = 'map'
    m.robot_id = robot_id
    m.task_id = task_id
    m.nav_status = nav_status
    m.is_charging = False
    m.x, m.y, m.yaw = x, y, 1.57
    m.battery_percent = battery
    m.battery_voltage = 12.1
    m.us_range_m = 0.42
    return m


def _make_ddagi(robot_id):
    m = DdagiTelemetry()
    m.header.frame_id = 'base_link'
    m.robot_id = robot_id
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


def _make_fleet():
    """ddago 2대 + ddagi 1대(servo 7개)로 채운 FleetTelemetry."""
    fleet = FleetTelemetry()
    fleet.header.frame_id = 'automato'
    fleet.ddagos = [
        _make_ddago('dg_01', 1024, 3.21, 1.05, 78.5, 'NAVIGATING'),
        _make_ddago('dg_02', 0, 5.10, 2.30, 62.0, 'IDLE'),
    ]
    fleet.ddagis = [_make_ddagi('dg_01')]
    return fleet


@pytest.fixture
def ctx():
    """FleetTelemetryRelay 와 가짜 HQ/QT 헬퍼를 백그라운드 스핀."""
    rclpy.init()
    relay = FleetTelemetryRelay()
    helper = rclpy.create_node('test_fleet_relay_helper')

    executor = SingleThreadedExecutor()
    executor.add_node(relay)
    executor.add_node(helper)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()

    yield relay, helper

    executor.shutdown()
    relay.destroy_node()
    helper.destroy_node()
    rclpy.shutdown()


def test_relay_passthrough_no_field_loss(ctx):
    """HQ 원본 FleetTelemetry 가 QT 토픽으로 필드/배열 손실 없이 그대로 전달된다."""
    _relay, helper = ctx
    received = []
    helper.create_subscription(
        FleetTelemetry, OUT_TOPIC, lambda m: received.append(m), 10)
    pub = helper.create_publisher(FleetTelemetry, IN_TOPIC, 10)

    src = _make_fleet()

    def relayed():
        # 연결이 성립할 때까지 매 폴링마다 재발행(테스트 관례).
        pub.publish(src)
        return bool(received)

    assert _wait_until(relayed), '릴레이 출력 토픽에서 메시지를 못 받음'

    out = received[-1]

    # --- 배열 길이 보존 ---
    assert len(out.ddagos) == 2
    assert len(out.ddagis) == 1

    # --- Ddago 원본 필드 보존 ---
    assert out.ddagos[0].robot_id == 'dg_01'
    assert out.ddagos[0].task_id == 1024
    assert out.ddagos[0].nav_status == 'NAVIGATING'
    assert out.ddagos[0].x == pytest.approx(3.21, abs=1e-3)
    assert out.ddagos[0].battery_percent == pytest.approx(78.5, abs=1e-2)
    assert out.ddagos[0].header.frame_id == 'map'
    assert out.ddagos[1].robot_id == 'dg_02'
    assert out.ddagos[1].nav_status == 'IDLE'

    # --- Ddagi 원본 필드 보존 (고정 배열/중첩 메시지) ---
    ddagi = out.ddagis[0]
    assert ddagi.robot_id == 'dg_01'
    assert not ddagi.is_paused
    assert list(ddagi.joint_angles) == pytest.approx(
        [10.2, -30.5, 45.0, 0.0, -12.3, 5.5], abs=1e-3)
    assert len(ddagi.servo_health) == 7
    assert ddagi.servo_health[0].joint_no == 1
    assert ddagi.servo_health[6].gripper_value == 85   # 그리퍼 값 보존
    assert ddagi.servo_health[6].joint_no == 7

    # --- Fleet 헤더 보존 ---
    assert out.header.frame_id == 'automato'
