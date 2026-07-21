#!/usr/bin/env python3
"""RP-EX 자리 인계 — '서 있는 로봇이 잠깐이라도 투명인간이 되지 않는가'.

노드 자리를 예약 대상에 넣는 것만으로는 부족하다. 로봇이 한 지점에서 다음 지점으로
넘어가는 사이, 또는 다음 통로를 기다리며 서 있는 사이에 그 자리 예약이 끊기면 그
틈으로 남이 들어온다. 여기서 지키는 것:

  1. 구간(_navigate)과 구간 사이에 '서 있는 자리' 예약이 끊기지 않는다.
  2. 순찰이 끝나면 자리를 정확히 반납한다(누수 없음).
  3. 서서 기다리는 동안 TTL 이 지나도 자리를 뺏기지 않는다(대기 중 하트비트).
  4. 순찰 시작 시점부터 출발 지점 자리를 쥔다.

가짜 액션 클라이언트를 여기에 직접 둔다(verify_web 의 FakeNavigateClient 를 쓰지
않는다) — 테스트가 검증 도구에 의존하면 도구가 바뀔 때 같이 깨지기 때문.

테스트 그래프(일직선):  15 --c16-- 12 --c13-- 9 --c7-- 4

실행:
  PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pytest test/test_slot_handoff.py -v
"""
import os
import sys
import threading
import time
from concurrent.futures import Future

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from automato_control_service import patrol_dispatcher as pd            # noqa: E402
from automato_control_service.patrol_dispatcher import PatrolDispatcher  # noqa: E402
from automato_control_service.routing_engine import RoutingEngine        # noqa: E402

WAYPOINTS = [4, 9, 12, 15]
CORRIDORS = [
    {"corridor_id": 16, "a": 15, "b": 12},
    {"corridor_id": 13, "a": 12, "b": 9},
    {"corridor_id": 7,  "a": 9,  "b": 4},
]
WP_META = {n: {"x": n * 0.3, "y": 0.0, "yaw": 0.0, "capture": False}
           for n in WAYPOINTS}


class _Log:
    def __init__(self):
        self.lines = []

    def info(self, m):
        self.lines.append(m)

    def warn(self, m):
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
    """Navigate.Result 자리에 끼우는 최소 응답(코드 + 마지막 도달 노드)."""
    def __init__(self, code, last_wp):
        self.result = self
        self.result_code = code
        self.last_waypoint_id = last_wp


class FakeClient:
    """세그먼트를 '즉시 도착'으로 처리하는 최소 클라이언트.

    주행 시간은 검증 대상이 아니다 — 우리가 보는 것은 '자리 예약이 언제 끊기는가'라
    도착만 정확히 흉내 내면 된다. drive_delay 로 주행 중 상태를 관찰할 틈을 만든다.
    """

    def __init__(self, robot, drive_delay=0.0):
        self.robot = robot            # {"wp": 현재 노드} — 테스트가 위치를 읽는다
        self.drive_delay = drive_delay
        self.dispatched = []

    def wait_for_server(self, timeout_sec=None):
        return True

    def send_goal_async(self, goal, feedback_callback=None):
        wps = [w.waypoint_id for w in goal.waypoints]
        self.dispatched.append(wps)
        result_future = Future()

        def drive():
            for wp in wps:
                if self.drive_delay:
                    time.sleep(self.drive_delay)
                self.robot["wp"] = wp
                if feedback_callback is not None:
                    feedback_callback(_Feedback(wp))
            result_future.set_result(_Result(0, wps[-1]))

        threading.Thread(target=drive, daemon=True).start()
        goal_future = Future()
        goal_future.set_result(_Handle(result_future))
        return goal_future


class _Feedback:
    def __init__(self, wp):
        self.feedback = self
        self.current_waypoint_id = wp


# ------------------------------- 픽스처 ------------------------------- #
@pytest.fixture
def fast_timing(monkeypatch):
    """대기·하트비트를 짧게. from-import 로 바인딩된 모듈 상수를 직접 바꾼다."""
    monkeypatch.setattr(pd, "RESERVE_WAIT_SEC", 0.4)
    monkeypatch.setattr(pd, "RESERVE_POLL_SEC", 0.02)
    monkeypatch.setattr(pd, "HEARTBEAT_SEC", 0.02)


def _make(ttl=60.0):
    engine = RoutingEngine(WAYPOINTS, CORRIDORS, reservation_ttl=ttl)
    log = _Log()
    disp = PatrolDispatcher(log)
    disp.wp_meta = dict(WP_META)
    return engine, disp, log


def _slots_of(engine, robot_id):
    """robot_id 가 지금 쥐고 있는 '자리'의 노드 번호 집합."""
    return {n for n, rid in engine.reservation_snapshot()["nodes"].items()
            if rid == robot_id}


# --------------------------------------------------------------------------- #
def test_slot_never_drops_between_segments(fast_timing):
    """불변식 1·2 — 순찰 내내 자리를 하나는 쥐고 있고, 끝나면 정확히 반납한다."""
    engine, disp, log = _make()
    robot = {"wp": 15}
    client = FakeClient(robot, drive_delay=0.01)

    gaps = []
    stop = threading.Event()

    def watch():
        """'첫 예약을 본 뒤 ~ 종료 반납 전' 사이에 자리가 빈 순간을 센다.

        시작 전과 종료 후에 비어 있는 것은 정상이라 세지 않는다.
        """
        started = False
        while not stop.is_set():
            if log.has("순찰 종료"):
                return
            held = _slots_of(engine, "dg_01")
            if held:
                started = True
            elif started:
                gaps.append(robot["wp"])
            time.sleep(0.0005)

    t = threading.Thread(target=watch, daemon=True)
    t.start()
    result = disp.run_patrol(
        1, "dg_01", [{"waypoint_id": 12}, {"waypoint_id": 9}, {"waypoint_id": 4}],
        engine, client, start_wp=15)
    stop.set()
    t.join(timeout=2)

    assert result == "COMPLETED"
    assert not gaps, f"자리 예약이 {len(gaps)}회 끊겼다(그 틈에 남이 들어온다): {gaps}"
    assert _slots_of(engine, "dg_01") == set(), "순찰이 끝났는데 자리가 남았다"
    assert engine.reservation_snapshot()["corridors"] == {}, "통로 예약이 누수됐다"


def test_start_slot_claimed_at_patrol_start(fast_timing):
    """불변식 4 — 출발 지점 자리를 순찰 시작과 동시에 쥔다."""
    engine, disp, log = _make()
    client = FakeClient({"wp": 15})

    disp.run_patrol(1, "dg_01", [{"waypoint_id": 12}], engine, client, start_wp=15)
    assert log.has("출발 지점 15 자리 확보")


def test_standing_slot_survives_ttl_while_waiting(fast_timing):
    """불변식 3 — 서서 기다리는 동안 TTL 이 지나도 자리를 뺏기지 않는다.

    주행 중에는 _dispatch_segment 가 하트비트를 돌리지만, '서서 기다리는' 동안에는
    아무도 갱신하지 않는다. held 를 넘기지 않던 시절엔 여기서 자리를 잃었다.
    TTL 만료는 요청이 들어올 때 판정되므로, 남이 실제로 요청해 봐야 확인할 수 있다.
    """
    engine, disp, _log = _make(ttl=0.1)          # TTL 을 아주 짧게
    engine.try_reserve(engine.node_slot(9), "dg_01")     # dg_01 이 9번에 서 있고
    engine.try_reserve(13, "dg_02")                      # dg_02 가 통로13 을 안 놓는다

    stop_b = threading.Event()

    def keep_b():                                # dg_02 는 살아있는 로봇이다
        while not stop_b.is_set():
            engine.heartbeat(13, "dg_02")
            time.sleep(0.01)

    tb = threading.Thread(target=keep_b, daemon=True)
    tb.start()
    held = [engine.node_slot(9)]
    ok = disp._reserve_with_wait(engine, 13, "dg_01", held)   # 0.4초간 대기 → 양보
    stop_b.set()
    tb.join(timeout=1)

    assert ok is False, "남이 쥔 통로를 잡아버렸다"
    stolen = engine.try_reserve(engine.node_slot(9), "dg_03")
    assert stolen is False, "대기하는 사이 서 있는 자리를 뺏겼다(TTL 방어 실패)"


def test_standing_slot_lost_without_heartbeat(fast_timing):
    """대비군 — held 를 안 넘기면 실제로 뺏긴다(위 테스트가 헛돌지 않음을 보증)."""
    engine, disp, _log = _make(ttl=0.1)
    engine.try_reserve(engine.node_slot(9), "dg_01")
    engine.try_reserve(13, "dg_02")

    stop_b = threading.Event()

    def keep_b():
        while not stop_b.is_set():
            engine.heartbeat(13, "dg_02")
            time.sleep(0.01)

    tb = threading.Thread(target=keep_b, daemon=True)
    tb.start()
    disp._reserve_with_wait(engine, 13, "dg_01", None)        # held 없이
    stop_b.set()
    tb.join(timeout=1)

    assert engine.try_reserve(engine.node_slot(9), "dg_03") is True


def test_passed_slots_released_while_driving(fast_timing):
    """지나온 자리는 세그먼트가 끝나기 전에 반납된다 — 안 그러면 뒤 로봇이 다 막힌다."""
    engine, disp, log = _make()
    client = FakeClient({"wp": 15}, drive_delay=0.05)

    disp.run_patrol(1, "dg_01", [{"waypoint_id": 4}], engine, client, start_wp=15)

    assert log.has("조기 반납"), "주행 중 조기 반납이 한 번도 일어나지 않았다"
    freed = [ln for ln in log.lines if "조기 반납" in ln]
    assert any("-" in ln for ln in freed), f"자리(음수)가 조기 반납되지 않았다: {freed}"


def test_navigate_leaves_standing_slot_for_next_leg(fast_timing):
    """불변식 1의 결정적 검증 — _navigate 가 끝나도 '서 있는 자리'는 남긴다.

    구간과 구간 사이는 µs 단위라 시간 샘플링(위 watch 스레드)으로는 놓칠 수 있다.
    여기서는 한 구간을 끝낸 직후의 예약표를 직접 들여다본다:
      · 도착 지점 자리  → 다음 구간이 이어받아야 하므로 '남아 있어야' 한다
      · 지나온 자리·통로 → 반납돼 있어야 한다
    """
    engine, disp, _log = _make()
    client = FakeClient({"wp": 15})

    outcome, node = disp._navigate(engine, client, 1, "dg_01", 15, 12)

    assert (outcome, node) == ("arrived", 12)
    assert engine.holder_of(engine.node_slot(12)) == "dg_01", \
        "도착 지점 자리를 놓아버렸다 — 다음 구간이 시작되기 전에 남이 들어올 수 있다"
    assert engine.holder_of(engine.node_slot(15)) is None, "떠나온 자리가 남았다"
    assert engine.reservation_snapshot()["corridors"] == {}, "통로가 반납되지 않았다"


def test_next_leg_inherits_the_slot(fast_timing):
    """앞 구간이 남긴 자리를 다음 구간이 '내 것'으로 이어받는다(멱등 재예약)."""
    engine, disp, _log = _make()
    client = FakeClient({"wp": 15})

    disp._navigate(engine, client, 1, "dg_01", 15, 12)
    assert engine.holder_of(engine.node_slot(12)) == "dg_01"

    # 남이 그 자리를 못 가로챈다 — 인계가 끊기지 않았다는 뜻
    assert engine.try_reserve(engine.node_slot(12), "dg_02") is False

    outcome, node = disp._navigate(engine, client, 1, "dg_01", 12, 9)
    assert (outcome, node) == ("arrived", 9)
    assert engine.holder_of(engine.node_slot(9)) == "dg_01"
    assert engine.holder_of(engine.node_slot(12)) is None


# ----------------- 조기 반납 범위(지나온 자원을 얼마나 놓는가) ----------------- #
# 세그먼트:  13 --c17-- 16 --c22-- 17 --c18-- 14
_SEG_START, _SEG_WPS, _SEG_CIDS = 13, [16, 17, 14], [17, 22, 18]


@pytest.mark.parametrize("reached,expect_cids,expect_nodes", [
    (13, [],           []),            # 아직 출발점 — 놓을 것이 없다
    (16, [17],         [13]),          # 첫 노드 도착 → 직전 통로·자리까지 즉시 반납
    (17, [17, 22],     [13, 16]),
    (14, [17, 22, 18], [13, 16, 17]),  # 세그먼트 끝 — 도착 자리(14)만 남는다
])
def test_passed_resources_releases_everything_behind(reached, expect_cids, expect_nodes):
    """도착한 지점 뒤는 통로·자리를 모두 즉시 반납한다(직전 것도 붙들지 않는다).

    예전에는 로봇 길이를 감안해 '직전 통로 + 직전 자리'를 한 칸 남겼다. 그 여유는
    홉이 (통로, 도착 자리) 쌍이 된 뒤로 자리 예약이 대신한다 — 도착 자리를 내가 쥔 이상
    그 통로로는 어느 방향으로도 들어올 수 없다. 한 칸을 더 붙들면 같은 보호를 두 번
    하면서 남의 길만 막는다(실제로 다른 로봇이 못 움직이는 문제가 났다).
    """
    engine = RoutingEngine(WAYPOINTS, CORRIDORS)
    freed = PatrolDispatcher._passed_resources(
        engine, _SEG_START, _SEG_WPS, _SEG_CIDS, reached)

    expected = set(expect_cids) | {engine.node_slot(n) for n in expect_nodes}
    assert set(freed) == expected
    assert engine.node_slot(reached) not in freed, "지금 서 있는 자리를 놓아버렸다"


def test_passed_resources_unknown_node_releases_nothing():
    """모르는 노드(짝 촬영 id 등)면 아무것도 반납하지 않는다 — 안전한 쪽으로 실패."""
    engine = RoutingEngine(WAYPOINTS, CORRIDORS)
    assert PatrolDispatcher._passed_resources(
        engine, _SEG_START, _SEG_WPS, _SEG_CIDS, 999) == []
