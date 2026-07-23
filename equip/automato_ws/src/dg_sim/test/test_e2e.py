#!/usr/bin/env python3
"""DCS(dg_control) + 시뮬 4종(dg_sim) end-to-end 통합 테스트.

실제 팀원 코드 없이, dg_ai_sim(TCP)·ddago_sim·ddagi_sim·acs_sim 만으로 DCS 의
E0(텔레메트리 취합)·E1(경로 하달)·E2(capture 노드 분석→저장 루프) 전체가 도는지 검증한다.

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

from dg_control.dcs_node import DcsNode
from dg_sim import dg_ai_sim
from dg_sim.acs_sim import AcsSim
from dg_sim.ddago_sim import DdagoSim
from dg_sim.ddagi_sim import DdagiSim

AI_PORT = 9199   # 테스트 전용 포트(기본 9100과 충돌 회피)
NUM_WP = 6       # 경로 전체 노드 수
SEG = 3          # 구간 크기 → 2회에 나눠 하달
CAPTURE_IDS = [1, 3, 5]   # acs_sim 규칙: 홀수 waypoint 만 capture=true


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
    dcs = DcsNode(parameter_overrides=[
        Parameter('ai_target_file', value='/nonexistent/dg_ai_target.json'),
        Parameter('ai_default_endpoint', value='127.0.0.1:%d' % AI_PORT),
        Parameter('fleet_hz', value=5.0),
    ])
    ddagi = DdagiSim(parameter_overrides=[Parameter('auto_telemetry', value=True)])
    ddago = DdagoSim(parameter_overrides=[
        Parameter('move_delay', value=0.15), Parameter('auto_telemetry', value=True)])
    acs = AcsSim(parameter_overrides=[Parameter('auto_start', value=False)])

    ex = MultiThreadedExecutor(num_threads=8)
    for n in (dcs, ddagi, ddago, acs):
        ex.add_node(n)
    threading.Thread(target=ex.spin, daemon=True).start()
    time.sleep(1.0)   # 서버/구독 준비 대기

    yield {'dcs': dcs, 'ddago': ddago, 'ddagi': ddagi, 'acs': acs}

    ex.shutdown()
    for n in (dcs, ddagi, ddago, acs):
        n.destroy_node()
    rclpy.shutdown()


def test_full_patrol_loop(system):
    """순찰 1회: ACS가 경로를 구간(Waypoint[])으로 나눠 하달 → DCS 중계 → DdaGo 완주.
    capture=true 노드에서만 분석→SaveDetection 저장이 일어난다."""
    acs = system['acs']
    acs.send_patrol(num_waypoints=NUM_WP, seg_size=SEG)

    deadline = time.time() + 30.0
    while time.time() < deadline and not (
            acs.patrol_done and len(acs.saved) >= len(CAPTURE_IDS)):
        time.sleep(0.2)

    # capture=true 노드에서만 저장 — 통과 노드(capture=false)는 저장이 없어야 한다
    assert acs.capture_ids == CAPTURE_IDS
    saved_ids = sorted(s['waypoint_id'] for s in acs.saved)
    assert saved_ids == CAPTURE_IDS, 'SaveDetection 저장 대상 불일치: %s' % acs.saved

    # dg_ai_sim.make_result 규칙과 일치해야 함
    for i in CAPTURE_IDS:
        got = next(s for s in acs.saved if s['waypoint_id'] == i)
        exp = dg_ai_sim.make_result(i)
        assert got['ripe'] == exp['ripe_percent']
        assert got['unripe'] == exp['unripe_percent']
        assert got['rotten'] == exp['rotten_percent']
        assert got['disease'] == exp['disease_percent']
        # 병해충 라벨 이미지는 disease_percent >= 5 일 때만 실려 온다(E3)
        assert got['has_image'] == (exp['disease_percent'] >= dg_ai_sim.DISEASE_ALERT_PCT)

    # 순찰 완주 확인: 마지막 구간 result 성공 + 마지막 노드까지 도달
    assert acs.patrol_done, '순찰 미완료'
    assert acs.last_result is not None and acs.last_result.result_code == 0
    assert acs.last_waypoint_id == NUM_WP - 1, 'last_waypoint_id=%d' % acs.last_waypoint_id


def test_fleet_telemetry(system):
    """E0: DCS가 ddago/ddagi 텔레메트리를 묶어 RobotTelemetry로 ACS에 전달."""
    acs = system['acs']
    # ddago/ddagi 텔레메트리(1Hz)가 DCS를 거쳐 취합돼 올라올 때까지 대기
    deadline = time.time() + 8.0
    while time.time() < deadline and (
            acs.last_fleet is None
            or len(acs.last_fleet.ddagos) < 1
            or len(acs.last_fleet.ddagis) < 1):
        time.sleep(0.2)
    assert acs.fleet_count > 0, 'RobotTelemetry 수신 안 됨'
    assert acs.last_fleet is not None
    assert len(acs.last_fleet.ddagos) >= 1, 'ddago 텔레메트리 취합 안 됨'
    assert len(acs.last_fleet.ddagis) >= 1, 'ddagi 텔레메트리 취합 안 됨'
