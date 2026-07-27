#!/usr/bin/env python3
"""RP-116 E4/22-1/22-2 — 순찰 종료 후 복귀·도킹과 그 실패 분기 전부.

'전체 그림'의 모든 갈래를 덮는다:
  A. 주행/판정 (PatrolDispatcher, 가짜 engine/client)
     - 정상 완료 → (COMPLETED, [], last_wp)
     - 로봇 중단 보고(code=2) → FAILED_ABORTED
     - 갇힘 판정(_escapable / _stranded_after_block)
     - 갇힘 → FAILED_BLOCKED
     - 복귀 주행(drive_to_point): 전 구간 capture=false
     - 도킹(dock): 성공 / 마커 없음 / N_dock 재시도 후 실패
  B. 오케스트레이션 (PatrolControlNode 의 _return_and_dock/_immobilize/_notify_dock_failed)
     - 복귀+도킹 성공 → 예약 전부 해제
     - 도킹 실패 → DOCK_FAILED 알림, 진입 노드 자리 유지
     - 복귀 막힘 → IMMOBILIZED + BLOCKED_UNRECOVERABLE
     - 충전소 미등록 → 복귀 생략, 자리 반납

가짜 액션 클라이언트를 여기에 직접 둔다(test_slot_handoff 와 같은 방침 — 검증 도구에
의존하지 않는다). B 파트는 rclpy 를 쓰는 patrol_node 를 지연 import 해, ROS 가 없어도
A 파트는 수집·실행된다.

테스트 그래프(일직선):  22(충전소 진입) --c1-- 15 --c16-- 12 --c13-- 9(순찰) --c7-- 4(순찰)

실행:
  PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pytest test/test_e4_return.py -v
"""
import os
import sys
import threading
import time
from concurrent.futures import Future

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from automato_control_service import patrol_dispatcher as pd              # noqa: E402
from automato_control_service.patrol_dispatcher import PatrolDispatcher    # noqa: E402
from automato_control_service.routing_engine import RoutingEngine          # noqa: E402

# --------------------------------------------------------------------------- #
# 테스트 그래프
# --------------------------------------------------------------------------- #
NODES = [4, 9, 12, 15, 22]
CORRIDORS = [
    {"corridor_id": 1,  "a": 22, "b": 15},
    {"corridor_id": 16, "a": 15, "b": 12},
    {"corridor_id": 13, "a": 12, "b": 9},
    {"corridor_id": 7,  "a": 9,  "b": 4},
]
# 9·4 만 순찰 지점(capture=True). 22 는 충전소 진입, 15·12 는 경유 노드.
WP_META = {n: {"x": n * 0.3, "y": 0.0, "yaw": 0.0, "capture": n in (9, 4)}
           for n in NODES}

MARKER = {"marker_id": "24", "dictionary": "DICT_5X5_1000",
          "squares_x": 6, "squares_y": 5,
          "square_size_m": 0.024, "marker_size_m": 0.018,
          "dock_offset_x": 0.0, "dock_offset_y": 0.0, "dock_offset_yaw": 0.0}
CHARGE = {"task_point_id": "CHARGE_01", "waypoint_id": 22}


# --------------------------------------------------------------------------- #
# 가짜 도구
# --------------------------------------------------------------------------- #
class _Log:
    def __init__(self):
        self.lines = []

    def info(self, m):
        self.lines.append(m)

    def warn(self, m):
        self.lines.append(m)

    def error(self, m):
        self.lines.append(m)

    def debug(self, m):
        pass

    def has(self, needle):
        return any(needle in line for line in self.lines)


class _Handle:
    def __init__(self, fut):
        self.accepted = True
        self._fut = fut

    def get_result_async(self):
        return self._fut


class _NavResult:
    def __init__(self, code, last_wp):
        self.result = self
        self.result_code = code
        self.last_waypoint_id = last_wp


class _NavFeedback:
    def __init__(self, wp):
        self.feedback = self
        self.current_waypoint_id = wp


class FakeNavClient:
    """세그먼트를 즉시 도착으로 처리하는 Navigate 가짜 클라이언트. code 로 결과 지정."""

    def __init__(self, robot, code=0):
        self.robot = robot            # {"wp": 현재 노드}
        self.code = code
        self.dispatched = []          # [(wp_ids, capture_flags), ...]

    def wait_for_server(self, timeout_sec=None):
        return True

    def send_goal_async(self, goal, feedback_callback=None):
        wps = [w.waypoint_id for w in goal.waypoints]
        caps = [bool(w.capture) for w in goal.waypoints]
        self.dispatched.append((wps, caps))
        result_future = Future()

        def drive():
            for wp in wps:
                self.robot["wp"] = wp
                if feedback_callback is not None:
                    feedback_callback(_NavFeedback(wp))
            result_future.set_result(_NavResult(self.code, wps[-1]))

        threading.Thread(target=drive, daemon=True).start()
        goal_future = Future()
        goal_future.set_result(_Handle(result_future))
        return goal_future


class _DockResult:
    def __init__(self, code, msg=""):
        self.result = self
        self.result_code = code
        self.message = msg
        self.final_error_m = 0.0
        self.final_lateral_m = 0.0
        self.final_yaw_error = 0.0


class _DockFeedback:
    def __init__(self, phase):
        self.feedback = self
        self.phase = phase
        self.marker_detected = True
        self.distance_to_marker_m = 0.1


class FakeDockClient:
    """Dock 결과를 code 로 지정하는 가짜 클라이언트. calls 로 하달 횟수를 센다."""

    def __init__(self, code=0):
        self.code = code
        self.calls = 0

    def wait_for_server(self, timeout_sec=None):
        return True

    def send_goal_async(self, goal, feedback_callback=None):
        self.calls += 1
        if feedback_callback is not None:
            feedback_callback(_DockFeedback("REVERSING"))
        result_future = Future()
        result_future.set_result(
            _DockResult(self.code, "ok" if self.code == 0 else "dock fail"))
        goal_future = Future()
        goal_future.set_result(_Handle(result_future))
        return goal_future


# --------------------------------------------------------------------------- #
# 픽스처
# --------------------------------------------------------------------------- #
@pytest.fixture
def fast(monkeypatch):
    """대기·하트비트·T_block 을 짧게(from-import 로 바인딩된 모듈 상수를 직접 바꾼다)."""
    monkeypatch.setattr(pd, "RESERVE_WAIT_SEC", 0.1)
    monkeypatch.setattr(pd, "RESERVE_POLL_SEC", 0.02)
    monkeypatch.setattr(pd, "HEARTBEAT_SEC", 0.02)
    monkeypatch.setattr(pd, "BLOCK_GIVEUP_SEC", 0.15)


def _make(ttl=60.0):
    engine = RoutingEngine(NODES, CORRIDORS, reservation_ttl=ttl)
    disp = PatrolDispatcher(_Log())
    disp.wp_meta = dict(WP_META)
    return engine, disp


# =========================================================================== #
# A. 주행/판정 (PatrolDispatcher)
# =========================================================================== #
def test_run_patrol_completed_returns_last_wp(fast):
    """정상 순찰 → (COMPLETED, [], 마지막 위치). 반환은 3-튜플이다."""
    engine, disp = _make()
    client = FakeNavClient({"wp": 15})
    status, unvisited, last = disp.run_patrol(
        1, "dg_01", [{"waypoint_id": 9}, {"waypoint_id": 4}],
        engine, client, start_wp=15)
    assert status == "COMPLETED"
    assert unvisited == []
    assert last == 4


def test_run_patrol_aborted_returns_failed_aborted(fast):
    """로봇이 중단(result_code=2)을 보고하면 FAILED_ABORTED."""
    engine, disp = _make()
    client = FakeNavClient({"wp": 15}, code=2)
    status, _unvisited, _last = disp.run_patrol(
        1, "dg_01", [{"waypoint_id": 9}], engine, client, start_wp=15)
    assert status == "FAILED_ABORTED"


def test_escapable_open_and_blocked(fast):
    """_escapable: 나갈 길이 있으면 True, 남이 유일 경로를 막으면 False."""
    engine, disp = _make()
    # 아무도 안 막음 → 순찰 지점으로 갈 수 있다
    assert disp._escapable(engine, "dg_01", 15, [9, 4], set()) is True
    # dg_02 가 c13(12-9) 점유 → 9·4 격리(일직선이라 우회 없음)
    engine.try_reserve(13, "dg_02")
    assert disp._escapable(engine, "dg_01", 15, [9, 4], set()) is False
    # 남은 미방문 지점이 없으면 갈 곳도 없다 → False
    assert disp._escapable(engine, "dg_01", 15, [9, 4], {9, 4}) is False


def test_stranded_true_after_tblock(fast):
    """_stranded_after_block: 갇히면 T_block 만큼 버틴 뒤 True."""
    engine, disp = _make()
    engine.try_reserve(13, "dg_02")            # 9·4 격리
    t0 = time.monotonic()
    assert disp._stranded_after_block(engine, "dg_01", 15, [9, 4], set()) is True
    assert time.monotonic() - t0 >= 0.15 * 0.8  # 대략 T_block 만큼 기다렸다


def test_stranded_false_when_escapable(fast):
    """갈 길이 있으면 재시도 없이 곧바로 False."""
    engine, disp = _make()
    assert disp._stranded_after_block(engine, "dg_01", 15, [9, 4], set()) is False


def test_run_patrol_stranded_returns_failed_blocked(fast):
    """순찰 중 갇힘(남은 지점 전부 도달 불가) → FAILED_BLOCKED."""
    engine, disp = _make()
    engine.try_reserve(13, "dg_02")            # 9·4 로 가는 유일 통로 차단
    status, _unvisited, _last = disp.run_patrol(
        1, "dg_01", [{"waypoint_id": 9}, {"waypoint_id": 4}],
        engine, FakeNavClient({"wp": 15}), start_wp=15)
    assert status == "FAILED_BLOCKED"


def test_drive_to_point_arrives_without_capture(fast):
    """복귀 주행: 목적지 도달 + 전 구간 capture=false(순찰 지점을 지나가도 안 찍는다)."""
    engine, disp = _make()
    client = FakeNavClient({"wp": 4})
    outcome, pos = disp.drive_to_point(1, "dg_01", 4, 22, engine, client)
    assert (outcome, pos) == ("arrived", 22)
    # 경로가 순찰 지점 9 를 지나가지만 어떤 하달에도 capture=True 가 없어야 한다
    assert client.dispatched, "복귀 주행이 아무것도 하달하지 않았다"
    for _wps, caps in client.dispatched:
        assert not any(caps), f"복귀 중 촬영 플래그가 켜졌다: {client.dispatched}"


def test_dock_success(fast):
    """도킹 성공(result_code=0) → (True, 0, ...), 한 번만 하달."""
    engine, disp = _make()
    dock = FakeDockClient(code=0)
    ok, code, _msg = disp.dock(1, "dg_01", "CHARGE_01", MARKER, dock)
    assert ok is True and code == 0
    assert dock.calls == 1


def test_dock_marker_none_fails_without_moving(fast):
    """마커 미등록 → 즉시 실패, 로봇(Dock 서버)을 부르지 않는다."""
    engine, disp = _make()
    dock = FakeDockClient(code=0)
    ok, code, _msg = disp.dock(1, "dg_01", "CHARGE_01", None, dock)
    assert ok is False and code is None
    assert dock.calls == 0


def test_dock_retries_then_fails(fast):
    """도킹이 계속 실패하면 N_dock 회 재시도 후 (False, code)."""
    engine, disp = _make()
    dock = FakeDockClient(code=1)              # 매번 마커 미검출
    ok, code, _msg = disp.dock(1, "dg_01", "CHARGE_01", MARKER, dock)
    assert ok is False and code == 1
    assert dock.calls == pd.DOCK_RETRY_MAX


# =========================================================================== #
# B. 오케스트레이션 (PatrolControlNode 의 복귀·도킹 시퀀스)
# =========================================================================== #
class _FakeNode:
    """복귀·도킹 오케스트레이션 메서드를 언바운드로 호출하기 위한 최소 self."""

    def __init__(self, cls, dispatcher, nav_client, dock_client):
        self._cls = cls                   # PatrolControlNode (형제 메서드 위임용)
        self._db_pool = object()          # truthy — 실제 DB 호출은 monkeypatch 로 대체
        self._dispatcher = dispatcher
        self._web_url = "http://web"
        self._nav = nav_client
        self._dock = dock_client
        self._log = _Log()

    def get_logger(self):
        return self._log

    def _client_for(self, robot_id):
        return self._nav

    def _dock_client_for(self, robot_id):
        return self._dock

    # _return_and_dock 이 부르는 형제 메서드는 실제 구현으로 위임한다(self=이 가짜 노드).
    def _immobilize(self, *args, **kwargs):
        return self._cls._immobilize(self, *args, **kwargs)

    def _notify_dock_failed(self, *args, **kwargs):
        return self._cls._notify_dock_failed(self, *args, **kwargs)


@pytest.fixture
def orchestration(monkeypatch):
    """patrol_node 를 지연 import 하고, DB·알림 함수를 기록용으로 갈아끼운다.

    반환 dict 의 charge/marker 를 테스트가 미리 채운다. 나머지(immobilize/failed/events)
    에는 호출 기록이 쌓인다.
    """
    from automato_control_service import patrol_node as pn
    rec = {"charge": None, "marker": None,
           "immobilize": [], "failed": [], "events": []}

    monkeypatch.setattr(pn.automato_db, "get_charge_point",
                        lambda pool, rid: rec["charge"])
    monkeypatch.setattr(pn.automato_db, "get_dock_marker",
                        lambda pool, tpid: rec["marker"])
    monkeypatch.setattr(pn.automato_db, "set_operational_status",
                        lambda pool, rid, st: rec["immobilize"].append((rid, st)))

    def _save_event(pool, **kw):
        rec["events"].append(kw)
        return 1

    monkeypatch.setattr(pn.automato_db, "save_event_log", _save_event)

    def _send_failed(url, payload, **kw):
        rec["failed"].append(payload)
        return True

    monkeypatch.setattr(pn.patrol_notify, "send_task_failed", _send_failed)

    rec["_cls"] = pn.PatrolControlNode
    return rec


def test_return_and_dock_success_releases_all(fast, orchestration):
    """복귀+도킹 성공 → 이 로봇의 예약이 전부 비고, 도킹은 한 번만."""
    engine, disp = _make()
    orchestration["charge"] = CHARGE
    orchestration["marker"] = MARKER
    node = _FakeNode(orchestration["_cls"], disp,FakeNavClient({"wp": 4}), FakeDockClient(code=0))

    orchestration["_cls"]._return_and_dock(node, 1, "dg_01", engine, 4)

    snap = engine.reservation_snapshot()
    assert snap["nodes"] == {} and snap["corridors"] == {}, \
        f"도킹 성공 후에도 예약이 남았다: {snap}"
    assert node._dock.calls == 1
    assert orchestration["failed"] == []       # 성공이라 실패 알림 없음


def test_return_and_dock_dock_failed_notifies(fast, orchestration):
    """복귀는 됐으나 도킹이 N_dock 소진 → DOCK_FAILED 알림 + 진입 노드 자리 유지."""
    engine, disp = _make()
    orchestration["charge"] = CHARGE
    orchestration["marker"] = MARKER
    node = _FakeNode(orchestration["_cls"], disp,FakeNavClient({"wp": 4}), FakeDockClient(code=1))

    orchestration["_cls"]._return_and_dock(node, 1, "dg_01", engine, 4)

    assert node._dock.calls == pd.DOCK_RETRY_MAX
    assert any(p["reason"] == "DOCK_FAILED" for p in orchestration["failed"])
    # 관리자 개입 대기 — 진입 노드 자리는 놓지 않는다
    assert engine.holder_of(engine.node_slot(22)) == "dg_01"


def test_return_and_dock_blocked_immobilizes(fast, orchestration):
    """복귀 경로마저 막힘 → 22-2 현장 정지(IMMOBILIZED + BLOCKED_UNRECOVERABLE)."""
    engine, disp = _make()
    orchestration["charge"] = CHARGE
    orchestration["marker"] = MARKER
    engine.try_reserve(16, "dg_02")            # 15-12 통로 점유 → 22 로 가는 길 차단
    node = _FakeNode(orchestration["_cls"], disp,FakeNavClient({"wp": 4}), FakeDockClient(code=0))

    orchestration["_cls"]._return_and_dock(node, 1, "dg_01", engine, 4)

    assert ("dg_01", "IMMOBILIZED") in orchestration["immobilize"]
    assert any(p["reason"] == "BLOCKED_UNRECOVERABLE"
               for p in orchestration["failed"])
    assert any(e.get("event_type") == "TRAFFIC_CONTROL"
               for e in orchestration["events"])
    assert node._dock.calls == 0               # 도킹까지 가지도 못했다


def test_return_and_dock_no_charge_point_releases_slot(fast, orchestration):
    """전용 충전소 미등록 → 복귀 생략, 순찰이 넘긴 자리만 반납."""
    engine, disp = _make()
    orchestration["charge"] = None
    engine.try_reserve(engine.node_slot(4), "dg_01")   # 순찰이 쥔 채 넘긴 자리
    node = _FakeNode(orchestration["_cls"], disp,FakeNavClient({"wp": 4}), FakeDockClient(code=0))

    orchestration["_cls"]._return_and_dock(node, 1, "dg_01", engine, 4)

    assert engine.holder_of(engine.node_slot(4)) is None  # 자리 반납됨
    assert node._dock.calls == 0
    assert orchestration["failed"] == []
