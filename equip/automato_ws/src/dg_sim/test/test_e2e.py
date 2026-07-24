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


# ============================ S2 E2 수확 이동 + 도킹 ============================
HARVEST_WP = 4     # 수확 위치까지 노드 수
HARVEST_SEG = 2    # 구간 크기 → 2회에 나눠 하달


def _wait(cond, timeout=20.0, step=0.1):
    deadline = time.time() + timeout
    while time.time() < deadline and not cond():
        time.sleep(step)
    return cond()


def test_harvest_move_and_dock(system):
    """S2 E2: 수확 위치까지 이동(전 구간 capture=false) → 도착 후 도킹 → 성공.

    - 이동 중 촬영·분석이 전혀 없어야 한다(capture=false → AnalyzeFrame 미호출).
    - Dock feedback(phase)·result(오차 축별 값)가 DCS 를 거쳐 그대로 ACS 로 올라온다.
    - 도킹 성공 시에만 DCS 의 E3 진입 게이트(is_docked)가 열린다.
    """
    acs, dcs = system['acs'], system['dcs']
    task_id = acs.send_harvest_move(num_waypoints=HARVEST_WP, seg_size=HARVEST_SEG)
    assert task_id is not None

    assert _wait(lambda: acs.dock_done), '도킹 결과 미수신'

    # 이동은 끝났고 도킹은 성공(code 0)
    assert acs.harvest_move_done
    assert acs.last_dock_result is not None
    assert acs.last_dock_result.result_code == 0, acs.last_dock_result.message

    # 수확 이동은 촬영이 없다 → 분석·저장이 한 건도 없어야 한다
    assert acs.saved == [], '수확 이동 중 분석/저장이 발생함: %s' % acs.saved

    # Dock feedback(phase)이 중계됐다 — 탐색~후진까지의 단계가 올라온다
    assert acs.dock_feedback_phases, 'Dock feedback 미중계'
    assert 'SEARCHING' in acs.dock_feedback_phases

    # 오차 축별 값이 손실 없이 중계됐다(ddago_sim 성공값과 일치)
    assert abs(acs.last_dock_result.final_lateral_m - (-0.012)) < 1e-4
    assert abs(acs.last_dock_result.final_yaw_error - 0.021) < 1e-4

    # E3 진입 게이트: 도킹 성공한 task 만 열린다
    assert dcs.is_docked(task_id), '도킹 성공했는데 E3 게이트가 닫힘'
    assert not dcs.is_docked(task_id + 999), '엉뚱한 task 가 열림'


def test_dock_failure_no_marker(system):
    """도킹 실패(마커 미검출, code 1)가 DCS 를 거쳐 ACS 로 그대로 올라오고,
    실패한 task 는 E3 게이트가 열리지 않는다."""
    acs, dcs = system['acs'], system['dcs']
    system['ddago'].dock_mode = 'no_marker'
    task_id = acs.send_harvest_move(num_waypoints=HARVEST_WP, seg_size=HARVEST_SEG)

    assert _wait(lambda: acs.dock_done), '도킹 결과 미수신'
    assert acs.last_dock_result.result_code == 1, acs.last_dock_result.message
    assert not dcs.is_docked(task_id), '도킹 실패인데 E3 게이트가 열림'


def test_dock_failure_error_exceeded(system):
    """도킹 실패(정차 오차 초과, code 2)와 축별 오차 값이 그대로 중계된다."""
    acs, dcs = system['acs'], system['dcs']
    system['ddago'].dock_mode = 'error_exceeded'
    task_id = acs.send_harvest_move(num_waypoints=HARVEST_WP, seg_size=HARVEST_SEG)

    assert _wait(lambda: acs.dock_done), '도킹 결과 미수신'
    assert acs.last_dock_result.result_code == 2, acs.last_dock_result.message
    assert acs.last_dock_result.final_lateral_m > 0.05, '오차 값이 중계되지 않음'
    assert not dcs.is_docked(task_id)


def test_dock_cancel(system):
    """ACS 취소(E2 22-1)가 DCS 를 거쳐 DdaGo 까지 전파되어 도킹이 중단(code 3)된다."""
    acs, ddago, dcs = system['acs'], system['ddago'], system['dcs']
    ddago.move_delay = 1.2   # 취소를 걸 시간을 벌기 위해 도킹을 느리게
    task_id = acs.send_harvest_move(num_waypoints=HARVEST_WP, seg_size=HARVEST_SEG)

    # 도킹 feedback 이 흐르기 시작하면(=도킹 진행 중) 취소를 건다
    assert _wait(lambda: len(acs.dock_feedback_phases) >= 1, timeout=15.0), '도킹 시작 안 됨'
    acs.cancel_dock()

    assert _wait(lambda: acs.dock_done), '취소 결과 미수신'
    assert acs.last_dock_result.result_code == 3, acs.last_dock_result.message
    assert not dcs.is_docked(task_id), '취소됐는데 E3 게이트가 열림'


@pytest.fixture
def system_short_dock_timeout():
    """DCS 의 도킹 결과 대기 상한을 짧게(1.5s) 준 시스템 — 무응답 timeout 검증용."""
    rclpy.init()
    dcs = DcsNode(parameter_overrides=[
        Parameter('ai_target_file', value='/nonexistent/dg_ai_target.json'),
        Parameter('ai_default_endpoint', value='127.0.0.1:%d' % AI_PORT),
        Parameter('fleet_hz', value=5.0),
        Parameter('dock_result_timeout_sec', value=1.5),
    ])
    ddagi = DdagiSim(parameter_overrides=[Parameter('auto_telemetry', value=True)])
    ddago = DdagoSim(parameter_overrides=[
        Parameter('move_delay', value=0.15), Parameter('auto_telemetry', value=True)])
    acs = AcsSim(parameter_overrides=[Parameter('auto_start', value=False)])

    ex = MultiThreadedExecutor(num_threads=8)
    for n in (dcs, ddagi, ddago, acs):
        ex.add_node(n)
    threading.Thread(target=ex.spin, daemon=True).start()
    time.sleep(1.0)

    yield {'dcs': dcs, 'ddago': ddago, 'ddagi': ddagi, 'acs': acs}

    ex.shutdown()
    for n in (dcs, ddagi, ddago, acs):
        n.destroy_node()
    rclpy.shutdown()


def test_dock_timeout(system_short_dock_timeout):
    """DdaGo 가 도킹 결과를 안 주면(무응답) DCS 가 상한 시간 뒤 안전하게 실패(code 3)로
    ACS 에 돌려주고, 게이트는 열리지 않는다."""
    sys_ = system_short_dock_timeout
    acs, ddago, dcs = sys_['acs'], sys_['ddago'], sys_['dcs']
    ddago.dock_mode = 'hang'
    task_id = acs.send_harvest_move(num_waypoints=HARVEST_WP, seg_size=HARVEST_SEG)

    assert _wait(lambda: acs.dock_done, timeout=15.0), 'timeout 결과 미수신'
    assert acs.last_dock_result.result_code == 3, acs.last_dock_result.message
    assert not dcs.is_docked(task_id)
    acs.cancel_dock()   # 매달린 sim goal 을 풀어 teardown 을 빠르게
