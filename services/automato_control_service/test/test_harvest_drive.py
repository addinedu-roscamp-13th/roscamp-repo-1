#!/usr/bin/env python3
"""RP-123 E2 — 수확 로봇이 '순찰과 같은 규칙으로' 수확지까지 가서 도킹한다.

수확 주행은 새 알고리즘이 아니라 순찰이 쓰던 RouteRunner.drive 를 그대로 부르는 것이다.
그래서 여기서 볼 것은 주행 알고리즘 자체(그건 test_slot_handoff·test_node_occupancy 가
이미 지킨다)가 아니라, **디스패처가 그 엔진을 올바르게 여닫는가**이다:

  1. 수확지에 도달한다(순찰 훅 없이, 촬영도 짝도 없는 평범한 주행으로).
  2. 끝나면 예약이 하나도 안 남는다 — drive 는 '서 있는 자리'를 일부러 남기고
     나오므로, run_harvest 가 반납하지 않으면 그 자리가 영영 점유된 채 남는다.
     수확엔 순찰의 run_patrol 같은 뒷정리 주인이 따로 없어서 이게 유일한 방어선이다.
  3. 갈 수 없으면 FAILED 로 끊는다(그때도 예약은 안 남긴다).
  4. 출발점을 모르면 아예 안 나선다 — 순찰과 달리 폴백하지 않는다. 수확은 목적지가
     하나뿐이라 출발점을 모르면 경로 예약이 성립하지 않고, 도착 직후 ChArUco 도킹이
     붙어 위치가 어긋나면 그대로 도킹 실패가 된다.
  5. 도착하면 도킹한다(docking.dock 공용 모듈). 도킹 중에도 자리를 쥐고 있어야 하고
     (로봇이 거기 붙어 팔 작업을 한다), 실패하면 DOCK_FAILED 사유를 노드에 알려
     관리자 통지가 나가게 한다.

테스트 그래프(일직선):  15 --c16-- 12 --c13-- 9 --c7-- 4
  15 = 로봇 충전소 진입노드, 4 = 수확지(HARVEST_01) 진입노드

실행:
  PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pytest test/test_harvest_drive.py -v
"""
import os
import sys
import threading
from concurrent.futures import Future

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from automato_control_service import docking                             # noqa: E402
from automato_control_service import route_runner as rr                  # noqa: E402
from automato_control_service.harvest_dispatcher import (                # noqa: E402
    REASON_DOCK_FAILED,
    STATUS_FAILED,
    HarvestDispatcher,
)
from automato_control_service.routing_engine import RoutingEngine        # noqa: E402

WAYPOINTS = [4, 9, 12, 15]
CORRIDORS = [
    {"corridor_id": 16, "a": 15, "b": 12},
    {"corridor_id": 13, "a": 12, "b": 9},
    {"corridor_id": 7,  "a": 9,  "b": 4},
]
WP_META = {n: {"x": n * 0.3, "y": 0.0, "yaw": 0.0, "capture": False}
           for n in WAYPOINTS}
# 노드(_task_point_for)가 automato_db.get_task_point 로 읽어 넘겨주는 모양 그대로.
HARVEST_POINT = {"task_point_id": "HARVEST_01", "point_type": "HARVEST",
                 "waypoint_id": 4, "x": 1.2, "y": 0.0, "yaw": 0.0}
# 노드(_dock_marker_for)가 automato_db.get_dock_marker 로 읽어 넘겨주는 모양 그대로.
# 실제 시드 전에는 None 이 올 수 있고, 그때 도킹은 시도조차 하지 않는다(아래 테스트).
MARKER = {"marker_id": "31", "dictionary": "DICT_5X5_1000",
          "squares_x": 6, "squares_y": 5,
          "square_size_m": 0.024, "marker_size_m": 0.018,
          "dock_offset_x": 0.0, "dock_offset_y": 0.0, "dock_offset_yaw": 0.0}


class _Log:
    def __init__(self):
        self.lines = []

    def info(self, m):
        self.lines.append(m)

    def warn(self, m):
        self.lines.append(m)

    # 디스패처는 rclpy 로거의 warning/error 도 쓴다(로거 구현마다 이름이 다르다).
    def warning(self, m):
        self.lines.append(m)

    def error(self, m):
        self.lines.append(m)

    def debug(self, m):
        pass

    def has(self, needle):
        return any(needle in line for line in self.lines)


# --------------------------- 가짜 액션 클라이언트 --------------------------- #
class _Handle:
    def __init__(self, fut):
        self.accepted = True
        self._fut = fut

    def get_result_async(self):
        return self._fut


class _Result:
    def __init__(self, code, last_wp):
        self.result = self
        self.result_code = code
        self.last_waypoint_id = last_wp


class _Feedback:
    def __init__(self, wp):
        self.feedback = self
        self.current_waypoint_id = wp


class FakeNav:
    """세그먼트를 즉시 '도착'으로 처리하는 최소 Navigate 클라이언트.

    검증 도구(verify_web)의 것을 쓰지 않고 여기 직접 둔다 — 테스트가 도구에 의존하면
    도구가 바뀔 때 같이 깨진다(test_slot_handoff 와 같은 방침).
    """

    def __init__(self, server_up=True, code=0):
        self.server_up = server_up
        self.code = code          # 0 도착 / 1 막힘 / 2 중단
        self.dispatched = []

    def wait_for_server(self, timeout_sec=None):
        return self.server_up

    def send_goal_async(self, goal, feedback_callback=None):
        wps = [w.waypoint_id for w in goal.waypoints]
        self.dispatched.append(wps)
        result_future = Future()

        def drive():
            for wp in wps:
                if feedback_callback is not None:
                    feedback_callback(_Feedback(wp))
            result_future.set_result(_Result(self.code, wps[-1]))

        threading.Thread(target=drive, daemon=True).start()
        goal_future = Future()
        goal_future.set_result(_Handle(result_future))
        return goal_future


class _DockResult:
    def __init__(self, code, message):
        self.result = self
        self.result_code = code
        self.message = message


class _DockFeedback:
    def __init__(self, phase):
        self.feedback = self
        self.phase = phase
        self.marker_detected = True
        self.distance_to_marker_m = 0.3


class FakeDock:
    """Dock 결과를 code 로 지정하는 가짜 클라이언트. calls 로 하달 횟수를 센다."""

    def __init__(self, code=0, server_up=True):
        self.code = code                  # 0 성공 / 1 마커 미검출 / 2 정차오차 초과
        self.server_up = server_up
        self.calls = 0

    def wait_for_server(self, timeout_sec=None):
        return self.server_up

    def send_goal_async(self, goal, feedback_callback=None):
        self.calls += 1
        self.goal = goal                  # 테스트가 Goal 내용을 들여다본다
        if feedback_callback is not None:
            feedback_callback(_DockFeedback("REVERSING"))
        result_future = Future()
        result_future.set_result(
            _DockResult(self.code, "ok" if self.code == 0 else "dock fail"))
        goal_future = Future()
        goal_future.set_result(_Handle(result_future))
        return goal_future


@pytest.fixture
def fast_timing(monkeypatch):
    """대기·하트비트를 짧게.

    ⚠️ 상수마다 타깃 모듈이 다르다 — 주행·예약은 route_runner, 도킹은 docking 이
    각각 from-import 로 값을 바인딩했다. 엉뚱한 모듈에 패치하면 에러 없이 안 먹는다.
    """
    monkeypatch.setattr(rr, "RESERVE_WAIT_SEC", 0.3)
    monkeypatch.setattr(rr, "RESERVE_POLL_SEC", 0.02)
    monkeypatch.setattr(rr, "HEARTBEAT_SEC", 0.02)
    monkeypatch.setattr(docking, "HEARTBEAT_SEC", 0.02)


def _make(nav=None, dock=None):
    engine = RoutingEngine(WAYPOINTS, CORRIDORS, reservation_ttl=60.0)
    log = _Log()
    disp = HarvestDispatcher(log)
    disp.runner.wp_meta = dict(WP_META)
    return (engine, disp, log,
            (nav if nav is not None else FakeNav()),
            (dock if dock is not None else FakeDock()))


def _clients(nav, dock=None):
    """E2 가 쓰는 것은 nav·dock 뿐 — harvest/unload 는 아직 부르지 않는다
    (None 이라 실수로 부르면 즉시 AttributeError 로 드러난다)."""
    return {"nav": nav, "dock": dock, "harvest": None, "unload": None}


def _no_reservations(engine):
    snap = engine.reservation_snapshot()
    return snap["corridors"] == {} and snap["nodes"] == {}


# --------------------------------------------------------------------------- #
def test_수확지까지_주행하고_자리를_반납한다(fast_timing):
    """1·2 — 수확지에 도달하고, 끝나면 예약이 하나도 안 남는다."""
    engine, disp, log, nav, dock = _make()

    status, _reason = disp.run_harvest(
        1, "dg_01", HARVEST_POINT, MARKER, engine, _clients(nav, dock), start_wp=15)

    # 15 → 4 까지 실제로 하달됐는가(마지막 하달의 끝이 수확지여야 한다)
    assert nav.dispatched, "Navigate 하달이 한 번도 없었다"
    assert nav.dispatched[-1][-1] == 4, f"수확지까지 못 갔다: {nav.dispatched}"
    assert log.has("E2 수확지 도착"), f"도착 로그가 없다: {log.lines}"
    # E3~E6 이 아직 없으므로 수확을 안 한 채 끝난다 → tasks 는 FAILED 가 맞다.
    assert status == STATUS_FAILED
    assert _no_reservations(engine), \
        f"예약이 남았다(그 자리는 영영 점유된다): {engine.reservation_snapshot()}"


def test_촬영_없는_평범한_주행으로_하달된다(fast_timing):
    """수확은 순찰 훅을 안 쓴다 — 하달 배열에 촬영 플래그도 짝도 끼지 않는다."""
    engine, disp, _log, nav, dock = _make()

    disp.run_harvest(1, "dg_01", HARVEST_POINT, MARKER, engine,
                     _clients(nav, dock), start_wp=15)

    flat = [wp for seg in nav.dispatched for wp in seg]
    assert set(flat) <= set(WAYPOINTS), \
        f"그래프에 없는 노드(짝 등)가 하달됐다: {flat}"


def test_경로가_없으면_FAILED_이고_예약도_안_남는다(fast_timing):
    """3 — 남이 길목을 다 쥐고 있어 갈 수 없으면 끊는다(그때도 뒷정리는 한다)."""
    engine, disp, log, nav, dock = _make()
    # 유일한 통로(15-12)의 도착 자리를 남이 쥐고 있으면 이 일직선 그래프에선 우회로가 없다.
    assert engine.try_reserve(engine.node_slot(12), "dg_09") is True

    status, _reason = disp.run_harvest(
        1, "dg_01", HARVEST_POINT, MARKER, engine, _clients(nav, dock), start_wp=15)

    assert status == STATUS_FAILED
    assert log.has("E2 주행 실패"), f"실패 로그가 없다: {log.lines}"
    assert engine.reservation_snapshot()["nodes"] == {12: "dg_09"}, \
        "남의 예약을 건드렸거나 내 예약이 남았다"
    assert engine.reservation_snapshot()["corridors"] == {}, "통로 예약이 누수됐다"


def test_로봇이_중단을_보고하면_FAILED(fast_timing):
    """3 — 로봇이 스스로 멈췄다(result_code=2)면 재시도 없이 끊는다."""
    engine, disp, _log, nav, dock = _make(FakeNav(code=2))

    status, _reason = disp.run_harvest(
        1, "dg_01", HARVEST_POINT, MARKER, engine, _clients(nav, dock), start_wp=15)

    assert status == STATUS_FAILED
    assert _no_reservations(engine), "중단 보고 뒤에도 예약이 남았다"


def test_출발점을_모르면_나서지도_않는다(fast_timing):
    """4 — 순찰과 달리 폴백하지 않는다. 한 발짝도 움직이면 안 된다."""
    engine, disp, log, nav, dock = _make()

    status, _reason = disp.run_harvest(
        1, "dg_01", HARVEST_POINT, MARKER, engine, _clients(nav, dock), start_wp=None)

    assert status == STATUS_FAILED
    assert nav.dispatched == [], "출발점을 모르는데 로봇을 움직였다"
    assert log.has("출발 노드"), f"원인이 로그에 안 남았다: {log.lines}"
    assert _no_reservations(engine)


def test_그래프에_없는_수확지는_거절한다(fast_timing):
    """DB 에는 있지만 라우팅 그래프에 없는 지점 — 경로 계산이 불가능하다."""
    engine, disp, _log, nav, dock = _make()
    ghost = dict(HARVEST_POINT, waypoint_id=999)

    status, _reason = disp.run_harvest(
        1, "dg_01", ghost, MARKER, engine, _clients(nav, dock), start_wp=15)

    assert status == STATUS_FAILED
    assert nav.dispatched == []


def test_액션서버가_없으면_즉시_끊는다(fast_timing):
    """안 걸러내면 Goal 마다 수락 타임아웃을 다 기다린 뒤에야 실패한다."""
    engine, disp, _log, nav, dock = _make(FakeNav(server_up=False))

    status, _reason = disp.run_harvest(
        1, "dg_01", HARVEST_POINT, MARKER, engine, _clients(nav, dock), start_wp=15)

    assert status == STATUS_FAILED
    assert nav.dispatched == []


# ------------------------------ E2 ChArUco 도킹 ------------------------------ #
def test_도착하면_수확지_마커로_도킹한다(fast_timing):
    """5 — 도착 후 도킹을 한 번 하달하고, Goal 에 그 지점의 마커가 실린다."""
    engine, disp, log, nav, dock = _make()

    disp.run_harvest(1, "dg_01", HARVEST_POINT, MARKER, engine,
                     _clients(nav, dock), start_wp=15)

    assert dock.calls == 1, f"도킹 하달 횟수가 이상하다: {dock.calls}"
    # 충전소가 아니라 '이 수확지'로 도킹해야 한다 — 지점 id 를 잘못 실으면 로봇이
    # 엉뚱한 도크를 찾는다.
    assert dock.goal.task_point_id == "HARVEST_01"
    assert dock.goal.marker_id == MARKER["marker_id"]
    assert dock.goal.squares_x == MARKER["squares_x"]
    assert log.has("E2 도킹 완료"), f"도킹 완료 로그가 없다: {log.lines}"


def test_도킹_중에는_자리를_쥐고_있다(fast_timing):
    """도킹은 수십 초 걸린다 — 그동안 자리를 놓으면 남이 그 지점으로 들어온다.

    도킹 시점에 예약표를 들여다봐, 로봇이 수확지 자리를 쥔 채인지 확인한다.
    (끝난 뒤가 아니라 '도킹 중'이어야 의미가 있다 — 끝나면 finally 가 반납한다.)
    """
    engine, disp, _log, nav, dock = _make()
    seen = {}

    real_send = dock.send_goal_async

    def spy(goal, feedback_callback=None):
        seen["holder"] = engine.holder_of(engine.node_slot(4))
        return real_send(goal, feedback_callback)

    dock.send_goal_async = spy
    disp.run_harvest(1, "dg_01", HARVEST_POINT, MARKER, engine,
                     _clients(nav, dock), start_wp=15)

    assert seen.get("holder") == "dg_01", \
        f"도킹 중에 수확지 자리를 안 쥐고 있었다(홀더={seen.get('holder')})"


def test_마커가_없으면_도킹을_시도조차_안_한다(fast_timing):
    """마커 미시드(None)는 정상 상태다 — 값 없는 Goal 로 로봇을 움직이면 안 된다."""
    engine, disp, _log, nav, dock = _make()

    status, reason = disp.run_harvest(
        1, "dg_01", HARVEST_POINT, None, engine, _clients(nav, dock), start_wp=15)

    assert dock.calls == 0, "마커도 없이 Dock Goal 을 보냈다"
    assert status == STATUS_FAILED
    assert reason == REASON_DOCK_FAILED, "관리자 통지가 나가야 하는 실패다"
    assert _no_reservations(engine), "도킹 실패 뒤에도 예약이 남았다"


def test_도킹이_계속_실패하면_N회_재시도_후_DOCK_FAILED(fast_timing):
    """N_dock 소진 → FAILED + DOCK_FAILED 사유(노드가 이걸 보고 관리자에게 알린다)."""
    engine, disp, log, nav, dock = _make(dock=FakeDock(code=1))

    status, reason = disp.run_harvest(
        1, "dg_01", HARVEST_POINT, MARKER, engine, _clients(nav, dock), start_wp=15)

    assert dock.calls == docking.DOCK_RETRY_MAX, \
        f"재시도 횟수가 규격(N_dock)과 다르다: {dock.calls}"
    assert status == STATUS_FAILED
    assert reason == REASON_DOCK_FAILED
    assert log.has("E2 도킹 실패"), f"실패 로그가 없다: {log.lines}"
    assert _no_reservations(engine), "도킹 실패 뒤에도 예약이 남았다"


def test_주행_실패는_도킹까지_가지_않는다(fast_timing):
    """길이 막혀 못 갔으면 도킹 단계는 아예 없다(그리고 DOCK_FAILED 도 아니다)."""
    engine, disp, _log, nav, dock = _make()
    assert engine.try_reserve(engine.node_slot(12), "dg_09") is True

    status, reason = disp.run_harvest(
        1, "dg_01", HARVEST_POINT, MARKER, engine, _clients(nav, dock), start_wp=15)

    assert dock.calls == 0, "도착도 못 했는데 도킹을 시도했다"
    assert status == STATUS_FAILED
    assert reason is None, "주행 실패에 도킹 실패 사유가 붙었다"
