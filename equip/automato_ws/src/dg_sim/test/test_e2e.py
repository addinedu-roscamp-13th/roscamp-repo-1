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
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter

from automato_interfaces.action import Dock
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
        Parameter('harvest_feedback_timeout_sec', value=2.0),   # 워치독 테스트용 짧게
        Parameter('unload_feedback_timeout_sec', value=2.0),    # 하역 워치독 테스트용 짧게
    ])
    ddagi = DdagiSim(parameter_overrides=[
        Parameter('auto_telemetry', value=True),
        Parameter('harvest_step_delay', value=0.1),
        Parameter('unload_step_delay', value=0.05)])
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


def _wait(cond, timeout=20.0, step=0.1):
    deadline = time.time() + timeout
    while time.time() < deadline and not cond():
        time.sleep(step)
    return cond()


def _ddago_tel(acs):
    """DCS 가 취합해 ACS 로 올린 fleet 텔레메트리에서 ddago 항목 1개(없으면 None)."""
    fleet = acs.last_fleet
    if fleet is None or not fleet.ddagos:
        return None
    return fleet.ddagos[0]


def test_nav_status_returns_to_idle(system):
    """E0/E1 회귀: 주행이 끝나면 nav_status 가 IDLE 로 돌아와 다음 작업을 받을 수 있다.

    예전엔 nav_status 를 task_id 에서 유도해(task_id != 0 → NAVIGATING) 주행이 끝나도
    '작업 중'이 내려가지 않았다. DCS 가용 판정이 nav_status=='IDLE' 을 AND 조건으로
    요구하므로(telemetry_ws/patrol_api) 로봇이 영영 ROBOT_BUSY 로 남아, 시뮬을 껐다
    켜야만 두 번째 작업을 받을 수 있었다.

    반대로 task_id 는 goal 이 끝나도 유지되어야 한다 — 실물(ddago_control/
    telemetry_publisher)이 지키는 규약으로, 구간 사이마다 0 이 되면 QT 화면이 깜빡이고
    복귀 주행(같은 task_id 재사용) 추적이 끊긴다.
    """
    acs = system['acs']

    task1 = acs.send_patrol(num_waypoints=NUM_WP, seg_size=SEG)
    assert _wait(lambda: acs.patrol_done, timeout=30.0), '1차 순찰 미완료'

    # 주행 종료 → '움직이는 중' 표시가 내려간다(텔레메트리 1Hz + DCS 취합 지연 감안)
    idle = _wait(lambda: _ddago_tel(acs) is not None
                 and _ddago_tel(acs).nav_status == 'IDLE', timeout=8.0)
    tel = _ddago_tel(acs)
    assert idle, ('주행 종료 후에도 nav_status 가 IDLE 로 복귀하지 않음: %s'
                  % (tel.nav_status if tel else None))

    # task_id 는 마지막 값 그대로 유지(0 으로 되돌리지 않는다)
    assert tel.task_id == task1, 'task_id 가 유지되지 않음: %d (기대 %d)' % (tel.task_id, task1)

    # 두 번째 작업을 실제로 받아 완주한다(재시작 없이)
    task2 = acs.send_patrol(num_waypoints=NUM_WP, seg_size=SEG)
    assert task2 != task1, '두 번째 순찰이 발행되지 않음(ROBOT_BUSY 로 막힘)'
    assert _wait(lambda: acs.patrol_done, timeout=30.0), '2차 순찰 미완료'
    assert acs.last_result.result_code == 0, acs.last_result.message


def test_dock_sets_task_id(system):
    """도킹 goal 도 텔레메트리 task_id 를 갱신한다 — 실물 dock_server 가
    /ddago/current_task 로 알리는 것과 같은 규약. 주행 없이 도킹만 하달해 확인한다.

    nav_status 는 도킹 중에도 IDLE 이다(실물도 마찬가지 — 도킹 기동은 Nav2 goal 이 아니라
    dock_server 가 cmd_vel 을 직접 내므로 Nav2 상태에 안 잡힌다). 도킹 중 배차를 막는 것은
    nav_status 가 아니라 DB 의 활성 task 다.
    """
    acs = system['acs']
    cli = ActionClient(acs, Dock, '/ddago/dock')
    assert cli.wait_for_server(timeout_sec=5.0), 'ddago_sim Dock 서버 없음'
    try:
        goal = Dock.Goal()
        goal.task_id = 4242              # 순찰 task_id 와 겹치지 않는 값
        goal.task_point_id = 'HARVEST_01'

        send_fut = cli.send_goal_async(goal)
        assert _wait(send_fut.done, timeout=10.0), 'Dock goal 응답 없음'
        gh = send_fut.result()
        assert gh.accepted, 'Dock goal 거부'

        res_fut = gh.get_result_async()
        assert _wait(res_fut.done, timeout=20.0), 'Dock 결과 미수신'
        assert res_fut.result().result.result_code == 0
    finally:
        cli.destroy()

    assert _wait(lambda: _ddago_tel(acs) is not None
                 and _ddago_tel(acs).task_id == 4242, timeout=8.0), \
        '도킹 task_id 가 텔레메트리에 실리지 않음: %s' % (
            _ddago_tel(acs).task_id if _ddago_tel(acs) else None)
    assert _ddago_tel(acs).nav_status == 'IDLE', '도킹은 nav_status 를 바꾸지 않아야 한다'


# ============================ S2 E2 수확 이동 + 도킹 ============================
HARVEST_WP = 4     # 수확 위치까지 노드 수
HARVEST_SEG = 2    # 구간 크기 → 2회에 나눠 하달


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


# ============================ S2 E3 Harvest 중계 ============================
def _dock_first(system):
    """수확 이동+도킹으로 게이트를 연 뒤 task_id 반환(E3 진입 준비)."""
    acs = system['acs']
    tid = acs.send_harvest_move(num_waypoints=HARVEST_WP, seg_size=HARVEST_SEG)
    assert _wait(lambda: acs.dock_done), '도킹 결과 미수신'
    assert acs.last_dock_result.result_code == 0, '선행 도킹 실패'
    assert system['dcs'].is_docked(tid), '도킹 게이트 안 열림'
    return tid


def test_harvest_gate_reject(system):
    """도킹 안 된 task 로 Harvest 를 하달하면 DG 가 goal 을 거부(reject)한다."""
    acs, dcs = system['acs'], system['dcs']
    assert not dcs.is_docked(999999)
    acs.send_harvest_action(999999)
    assert _wait(lambda: acs.harvest_done), '수확 goal 응답 없음'
    assert acs.harvest_accepted is False, '도킹 안 된 task 인데 accept 됨'


def test_harvest_depleted(system):
    """도킹 성공 task 로 수확 → 라운드 Feedback 이 중계되고 DEPLETED Result 가 올라온다.
    수확 시작 시 도킹 게이트가 소비(해제)된다."""
    acs, dcs = system['acs'], system['dcs']
    tid = _dock_first(system)
    acs.send_harvest_action(tid, max_capacity=7)

    assert _wait(lambda: acs.harvest_done), '수확 결과 미수신'
    assert acs.harvest_accepted is True
    assert acs.harvest_result is not None
    assert acs.harvest_result.exit_reason == 'DEPLETED', acs.harvest_result.message

    # 라운드 Feedback 이 중계됐다 — 라운드가 진행되고 누적 카운트가 오른다
    assert acs.harvest_feedback, 'Harvest feedback 미중계'
    rounds_seen = {f['round'] for f in acs.harvest_feedback}
    assert max(rounds_seen) >= 2, '라운드 진행이 안 보임: %s' % rounds_seen
    assert acs.harvest_result.normal_count == 6   # 2라운드 × 3파지 (sim 기본)

    # 수확 시작으로 게이트가 소비됐다(재진입 방지)
    assert not dcs.is_docked(tid), '수확 시작 후에도 게이트가 열려 있음'


def test_harvest_full(system):
    """만차(FULL) 종료 사유가 그대로 중계된다. max_capacity 를 낮춰 도달시킨다."""
    acs = system['acs']
    system['ddagi'].harvest_mode = 'full'
    tid = _dock_first(system)
    acs.send_harvest_action(tid, max_capacity=4)   # 파지 6회 중 4개째 만차

    assert _wait(lambda: acs.harvest_done), '수확 결과 미수신'
    assert acs.harvest_result.exit_reason == 'FULL', acs.harvest_result.message
    assert acs.harvest_result.normal_count >= 4


def test_harvest_max_rounds(system):
    """MAX_ROUNDS_EXCEEDED 종료 사유가 그대로 중계된다."""
    acs = system['acs']
    system['ddagi'].harvest_mode = 'max_rounds'
    tid = _dock_first(system)
    acs.send_harvest_action(tid)

    assert _wait(lambda: acs.harvest_done), '수확 결과 미수신'
    assert acs.harvest_result.exit_reason == 'MAX_ROUNDS_EXCEEDED', acs.harvest_result.message


def test_harvest_cancel(system):
    """ACS 취소가 DG 를 거쳐 Ddagi 까지 전파되어 수확이 중단된다."""
    acs = system['acs']
    system['ddagi'].harvest_step_delay = 0.4   # 취소 걸 시간 확보(라운드가 천천히)
    tid = _dock_first(system)
    acs.send_harvest_action(tid)

    # Feedback 이 흐르기 시작하면(=수확 진행 중) 취소
    assert _wait(lambda: len(acs.harvest_feedback) >= 1, timeout=15.0), '수확 시작 안 됨'
    acs.cancel_harvest()

    assert _wait(lambda: acs.harvest_done), '취소 결과 미수신'
    # 취소로 끝나면 exit_reason 은 비어 있다(정상 종료 사유가 아님)
    assert acs.harvest_result is not None
    assert acs.harvest_result.exit_reason == '', acs.harvest_result.exit_reason


def test_harvest_watchdog(system):
    """Ddagi 가 진행 소식(Feedback)을 안 주면 DG 의 무수신 워치독이 안전하게 실패시킨다."""
    acs = system['acs']
    system['ddagi'].harvest_mode = 'hang'
    tid = _dock_first(system)
    acs.send_harvest_action(tid)

    # DCS harvest_feedback_timeout=2.0 → 워치독이 곧 실패로 마감
    assert _wait(lambda: acs.harvest_done, timeout=15.0), '워치독 결과 미수신'
    assert acs.harvest_accepted is True          # goal 은 수락됐다가
    assert acs.harvest_result is not None
    assert acs.harvest_result.exit_reason == ''  # 정상 종료가 아님(abort)
    assert 'watchdog' in acs.harvest_result.message
    acs.cancel_harvest()   # 매달린 sim goal 정리


# ============================ S2 E6 Unload(하역) 중계 ============================
def test_unload_gate_reject(system):
    """예냉실 도킹 안 된 task 로 Unload 를 하달하면 DG 가 goal 을 거부(reject)한다."""
    acs, dcs = system['acs'], system['dcs']
    assert not dcs.is_docked(888888)
    acs.send_unload_action(888888)
    assert _wait(lambda: acs.unload_done), '하역 goal 응답 없음'
    assert acs.unload_accepted is False, '도킹 안 된 task 인데 accept 됨'


def test_unload_success(system):
    """예냉실 도킹 성공 task 로 하역 → phase Feedback 이 순서대로 중계되고
    result_code 0(성공) Result 가 올라온다. 하역 시작 시 도킹 게이트가 소비된다."""
    acs, dcs = system['acs'], system['dcs']
    tid = _dock_first(system)
    acs.send_unload_action(tid, shake_delay_sec=0.1)   # WAIT 단계를 짧게

    assert _wait(lambda: acs.unload_done), '하역 결과 미수신'
    assert acs.unload_accepted is True
    assert acs.unload_result is not None
    assert acs.unload_result.result_code == 0, acs.unload_result.message

    # phase Feedback 이 순서대로 중계됐다(손잡이 파지 → … → 복귀)
    assert acs.unload_feedback, 'Unload feedback 미중계'
    assert acs.unload_feedback == ['GRIP_HANDLE', 'LIFT', 'WAIT', 'SHAKE', 'RETURN'], \
        '하역 phase 순서/누락: %s' % acs.unload_feedback

    # 하역 시작으로 게이트가 소비됐다(재진입 방지)
    assert not dcs.is_docked(tid), '하역 시작 후에도 게이트가 열려 있음'


def test_unload_grip_fail(system):
    """손잡이 파지 실패(result_code 1)가 DG 를 거쳐 그대로 ACS 로 올라온다.
    DG 는 실패를 성공으로 바꾸지 않는다(task FAILED 판정은 ACS 몫)."""
    acs = system['acs']
    system['ddagi'].unload_mode = 'grip_fail'
    tid = _dock_first(system)
    acs.send_unload_action(tid, shake_delay_sec=0.1)

    assert _wait(lambda: acs.unload_done), '하역 결과 미수신'
    assert acs.unload_result is not None
    assert acs.unload_result.result_code == 1, acs.unload_result.message


def test_unload_cancel(system):
    """ACS 취소가 DG 를 거쳐 Ddagi 까지 전파되어 하역이 중단(code 2)된다."""
    acs = system['acs']
    system['ddagi'].unload_step_delay = 0.4   # 취소 걸 시간 확보(phase 가 천천히)
    tid = _dock_first(system)
    acs.send_unload_action(tid, shake_delay_sec=0.1)

    # Feedback 이 흐르기 시작하면(=하역 진행 중) 취소
    assert _wait(lambda: len(acs.unload_feedback) >= 1, timeout=15.0), '하역 시작 안 됨'
    acs.cancel_unload()

    assert _wait(lambda: acs.unload_done), '취소 결과 미수신'
    assert acs.unload_result is not None
    assert acs.unload_result.result_code == 2, acs.unload_result.message


def test_unload_watchdog(system):
    """Ddagi 가 진행 소식(phase Feedback)을 안 주면 DG 의 무수신 워치독이 안전하게
    실패(code 2)시킨다."""
    acs = system['acs']
    system['ddagi'].unload_mode = 'hang'
    tid = _dock_first(system)
    acs.send_unload_action(tid, shake_delay_sec=0.1)

    # DCS unload_feedback_timeout=2.0 → 워치독이 곧 실패로 마감
    assert _wait(lambda: acs.unload_done, timeout=15.0), '워치독 결과 미수신'
    assert acs.unload_accepted is True           # goal 은 수락됐다가
    assert acs.unload_result is not None
    assert acs.unload_result.result_code == 2    # 중단(abort)
    assert 'watchdog' in acs.unload_result.message
    acs.cancel_unload()   # 매달린 sim goal 정리
