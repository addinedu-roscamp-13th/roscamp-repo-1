#!/usr/bin/env python3
"""HQ(dg_control) + 시뮬 4종(dg_sim) end-to-end 통합 테스트.

실제 팀원 코드 없이, dg_ai_sim(TCP)·ddago_sim·ddagi_sim·acs_sim 만으로 HQ 의
E0(텔레메트리 취합)·E1(순찰 하달)·E2(분석→저장 루프) 전체가 도는지 검증한다.

실행 (SETUP.md 규약):
  source /opt/ros/jazzy/setup.bash
  source install/setup.bash
  PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pytest src/dg_sim/test/test_e2e.py -v
"""
import os
import threading
import time

import pytest
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter

from dg_control.hq_node import HqNode
from dg_sim import dg_ai_sim
from dg_sim.acs_sim import AcsSim
from dg_sim.ddago_sim import DdagoSim
from dg_sim.ddagi_sim import DdagiSim

AI_PORT = 9199   # 테스트 전용 포트(기본 9100과 충돌 회피)
NUM_WP = 4


@pytest.fixture(scope='module', autouse=True)
def ai_server():
    # AI 시뮬 TCP 서버는 모듈당 1회만 기동(테스트마다 재바인딩 방지)
    os.environ['DG_AI_SIM_HOST'] = '127.0.0.1'
    os.environ['DG_AI_SIM_PORT'] = str(AI_PORT)
    os.environ['DG_AI_SIM_DELAY'] = '0'   # 테스트는 지연 없이 빠르게
    threading.Thread(target=dg_ai_sim.main, daemon=True).start()
    time.sleep(0.5)
    yield


@pytest.fixture
def system():
    rclpy.init()
    hq = HqNode(parameter_overrides=[
        Parameter('ai_target_file', value='/nonexistent/dg_ai_target.json'),
        Parameter('ai_default_endpoint', value='127.0.0.1:%d' % AI_PORT),
        Parameter('fleet_hz', value=5.0),
    ])
    ddagi = DdagiSim(parameter_overrides=[Parameter('auto_telemetry', value=True)])
    ddago = DdagoSim(parameter_overrides=[
        Parameter('move_delay', value=0.15), Parameter('auto_telemetry', value=True)])
    acs = AcsSim(parameter_overrides=[Parameter('auto_start', value=False)])

    ex = MultiThreadedExecutor(num_threads=8)
    for n in (hq, ddagi, ddago, acs):
        ex.add_node(n)
    threading.Thread(target=ex.spin, daemon=True).start()
    time.sleep(1.0)   # 서버/구독 준비 대기

    yield {'hq': hq, 'ddago': ddago, 'ddagi': ddagi, 'acs': acs}

    ex.shutdown()
    for n in (hq, ddagi, ddago, acs):
        n.destroy_node()
    rclpy.shutdown()


def test_full_patrol_loop(system):
    """순찰 1회: ACS가 waypoint를 하나씩 하달·순회하며 4개 전부 분석→저장까지 완주."""
    acs = system['acs']
    acs.send_patrol(num_waypoints=NUM_WP)

    # ACS 루프가 마지막 waypoint까지 완료(patrol_done)될 때까지 대기
    deadline = time.time() + 30.0
    while time.time() < deadline and not (acs.patrol_done and len(acs.saved) >= NUM_WP):
        time.sleep(0.2)

    assert len(acs.saved) == NUM_WP, 'SaveDetection 저장 수 부족: %s' % acs.saved

    # dg_ai_sim.make_result 규칙과 일치해야 함
    for i in range(NUM_WP):
        got = next(s for s in acs.saved if s['waypoint_id'] == i)
        exp = dg_ai_sim.make_result(i)
        assert got['ripe'] == exp['ripe_percent']
        assert got['unripe'] == exp['unripe_percent']
        assert got['rotten'] == exp['rotten_percent']
        assert got['disease'] == exp['disease_percent']

    # 순찰 완주 확인: 마지막 waypoint 결과 성공 + 전 waypoint 방문
    assert acs.patrol_done, '순찰 미완료'
    assert acs.last_visited == NUM_WP, 'visited=%d != %d' % (acs.last_visited, NUM_WP)
    assert acs.last_result is not None and acs.last_result.result_code == 0


def test_fleet_telemetry(system):
    """E0: HQ가 ddago/ddagi 텔레메트리를 취합해 FleetTelemetry로 ACS에 전달."""
    acs = system['acs']
    # ddago/ddagi 텔레메트리(1Hz)가 HQ를 거쳐 취합돼 올라올 때까지 대기
    deadline = time.time() + 8.0
    while time.time() < deadline and (
            acs.last_fleet is None
            or len(acs.last_fleet.ddagos) < 1
            or len(acs.last_fleet.ddagis) < 1):
        time.sleep(0.2)
    assert acs.fleet_count > 0, 'FleetTelemetry 수신 안 됨'
    assert acs.last_fleet is not None
    assert len(acs.last_fleet.ddagos) >= 1, 'ddago 텔레메트리 취합 안 됨'
    assert len(acs.last_fleet.ddagis) >= 1, 'ddagi 텔레메트리 취합 안 됨'
