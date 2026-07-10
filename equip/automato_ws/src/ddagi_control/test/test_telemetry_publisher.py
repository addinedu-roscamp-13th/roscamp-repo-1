#!/usr/bin/env python3
"""시나리오1 E0  Ddagi 텔레메트리 퍼블리셔 통합 테스트.

퍼블리셔 노드와 구독자 노드를 한 프로세스에서 함께 스핀시켜
/{robot_id}/ddagi/telemetry 토픽에 DdagiTelemetry 가 정상 발행되는지 검증한다.

실행:
  source /opt/ros/jazzy/setup.bash
  cd equip/automato_ws && source install/setup.bash
  pytest src/ddagi_control/test/test_telemetry_publisher.py -v
"""
import threading
import time

import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor

from automato_interfaces.msg import DdagiTelemetry
from ddagi_control.telemetry_publisher import TelemetryPublisher


def _wait_for(predicate, timeout=5.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        if predicate():
            return True
        time.sleep(0.05)
    return False


@pytest.fixture
def ros_ctx():
    """서버(1Hz 기본값) + 구독자 노드를 백그라운드 스레드에서 스핀."""
    rclpy.init()
    server = TelemetryPublisher()
    client_node = rclpy.create_node('test_telemetry_subscriber')

    executor = SingleThreadedExecutor()
    executor.add_node(server)
    executor.add_node(client_node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()

    yield server, client_node

    executor.shutdown()
    server.destroy_node()
    client_node.destroy_node()
    rclpy.shutdown()


def _subscribe(client_node, robot_id='dg_01'):
    received = []
    client_node.create_subscription(
        DdagiTelemetry, f'/{robot_id}/ddagi/telemetry', received.append, 10)
    return received


def test_publishes_telemetry_with_full_servo_health(ros_ctx):
    """토픽에 robot_id 와 7개 관절(6관절+그리퍼)의 servo_health 가 발행된다."""
    _, client_node = ros_ctx
    received = _subscribe(client_node)

    assert _wait_for(lambda: len(received) >= 1), '텔레메트리가 발행되지 않음'
    msg = received[0]
    assert msg.robot_id == 'dg_01'
    assert len(msg.servo_health) == 7
    assert msg.is_paused is False


def test_gripper_health_is_joint_seven(ros_ctx):
    """servo_health 의 7번째 원소가 그리퍼(gripper_value 포함)여야 한다."""
    _, client_node = ros_ctx
    received = _subscribe(client_node)

    assert _wait_for(lambda: len(received) >= 1), '텔레메트리가 발행되지 않음'
    gripper = received[0].servo_health[6]
    assert gripper.joint_no == 7
    # 실물 그리퍼 값은 팔의 현재 개폐 상태에 따라 달라지므로 범위만 검증.
    assert 0 <= gripper.gripper_value <= 100


def test_publishes_periodically(ros_ctx):
    """일정 주기로 여러 번 발행되어야 한다(기본 1Hz → 3.5초 내 3회 이상)."""
    _, client_node = ros_ctx
    received = _subscribe(client_node)

    assert _wait_for(lambda: len(received) >= 3, timeout=3.5), \
        f'주기적 발행 기대, 실제 {len(received)}회'
