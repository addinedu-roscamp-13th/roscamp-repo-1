#!/usr/bin/env python3
"""RP-123 E2 C-2 — 수확 로봇이 '순찰과 같은 규칙으로' 수확지까지 간다.

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
from automato_control_service import route_runner as rr                  # noqa: E402
from automato_control_service.harvest_dispatcher import (                # noqa: E402
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


@pytest.fixture
def fast_timing(monkeypatch):
    """대기·하트비트를 짧게. 상수를 바인딩한 모듈은 route_runner 다(C-1 이후)."""
    monkeypatch.setattr(rr, "RESERVE_WAIT_SEC", 0.3)
    monkeypatch.setattr(rr, "RESERVE_POLL_SEC", 0.02)
    monkeypatch.setattr(rr, "HEARTBEAT_SEC", 0.02)


def _make(nav=None):
    engine = RoutingEngine(WAYPOINTS, CORRIDORS, reservation_ttl=60.0)
    log = _Log()
    disp = HarvestDispatcher(log)
    disp.runner.wp_meta = dict(WP_META)
    return engine, disp, log, (nav if nav is not None else FakeNav())


def _clients(nav):
    """C-2 가 쓰는 것은 nav 뿐 — 나머지 셋은 아직 부르지 않는다(부르면 즉시 드러난다)."""
    return {"nav": nav, "dock": None, "harvest": None, "unload": None}


def _no_reservations(engine):
    snap = engine.reservation_snapshot()
    return snap["corridors"] == {} and snap["nodes"] == {}


# --------------------------------------------------------------------------- #
def test_수확지까지_주행하고_자리를_반납한다(fast_timing):
    """1·2 — 수확지에 도달하고, 끝나면 예약이 하나도 안 남는다."""
    engine, disp, log, nav = _make()

    status = disp.run_harvest(1, "dg_01", HARVEST_POINT, engine,
                              _clients(nav), start_wp=15)

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
    engine, disp, _log, nav = _make()

    disp.run_harvest(1, "dg_01", HARVEST_POINT, engine, _clients(nav), start_wp=15)

    flat = [wp for seg in nav.dispatched for wp in seg]
    assert set(flat) <= set(WAYPOINTS), \
        f"그래프에 없는 노드(짝 등)가 하달됐다: {flat}"


def test_경로가_없으면_FAILED_이고_예약도_안_남는다(fast_timing):
    """3 — 남이 길목을 다 쥐고 있어 갈 수 없으면 끊는다(그때도 뒷정리는 한다)."""
    engine, disp, log, nav = _make()
    # 유일한 통로(15-12)의 도착 자리를 남이 쥐고 있으면 이 일직선 그래프에선 우회로가 없다.
    assert engine.try_reserve(engine.node_slot(12), "dg_09") is True

    status = disp.run_harvest(1, "dg_01", HARVEST_POINT, engine,
                              _clients(nav), start_wp=15)

    assert status == STATUS_FAILED
    assert log.has("E2 주행 실패"), f"실패 로그가 없다: {log.lines}"
    assert engine.reservation_snapshot()["nodes"] == {12: "dg_09"}, \
        "남의 예약을 건드렸거나 내 예약이 남았다"
    assert engine.reservation_snapshot()["corridors"] == {}, "통로 예약이 누수됐다"


def test_로봇이_중단을_보고하면_FAILED(fast_timing):
    """3 — 로봇이 스스로 멈췄다(result_code=2)면 재시도 없이 끊는다."""
    engine, disp, _log, nav = _make(FakeNav(code=2))

    status = disp.run_harvest(1, "dg_01", HARVEST_POINT, engine,
                              _clients(nav), start_wp=15)

    assert status == STATUS_FAILED
    assert _no_reservations(engine), "중단 보고 뒤에도 예약이 남았다"


def test_출발점을_모르면_나서지도_않는다(fast_timing):
    """4 — 순찰과 달리 폴백하지 않는다. 한 발짝도 움직이면 안 된다."""
    engine, disp, log, nav = _make()

    status = disp.run_harvest(1, "dg_01", HARVEST_POINT, engine,
                              _clients(nav), start_wp=None)

    assert status == STATUS_FAILED
    assert nav.dispatched == [], "출발점을 모르는데 로봇을 움직였다"
    assert log.has("출발 노드"), f"원인이 로그에 안 남았다: {log.lines}"
    assert _no_reservations(engine)


def test_그래프에_없는_수확지는_거절한다(fast_timing):
    """DB 에는 있지만 라우팅 그래프에 없는 지점 — 경로 계산이 불가능하다."""
    engine, disp, _log, nav = _make()
    ghost = dict(HARVEST_POINT, waypoint_id=999)

    status = disp.run_harvest(1, "dg_01", ghost, engine, _clients(nav), start_wp=15)

    assert status == STATUS_FAILED
    assert nav.dispatched == []


def test_액션서버가_없으면_즉시_끊는다(fast_timing):
    """안 걸러내면 Goal 마다 수락 타임아웃을 다 기다린 뒤에야 실패한다."""
    engine, disp, _log, nav = _make(FakeNav(server_up=False))

    status = disp.run_harvest(1, "dg_01", HARVEST_POINT, engine,
                              _clients(nav), start_wp=15)

    assert status == STATUS_FAILED
    assert nav.dispatched == []
