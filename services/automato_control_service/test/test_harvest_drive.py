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
  6. 도킹 후 수확을 시킨다. 수확은 Ddagi 가 주관하고 ACS 는 기다리기만 하는데,
     **성공 판정을 goal 상태로** 해야 한다(Harvest 에는 result_code 가 없다).
     진행 보고는 콜백으로 노드에 넘긴다(디스패처는 HTTP 를 모른다).

테스트 그래프(일직선):  15 --c16-- 12 --c13-- 9 --c7-- 4
  15 = 로봇 충전소 진입노드, 4 = 수확지(HARVEST_01) 진입노드

실행:
  PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pytest test/test_harvest_drive.py -v
"""
import math
import os
import sys
import threading
from concurrent.futures import Future

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from action_msgs.msg import GoalStatus                                   # noqa: E402
from automato_control_service import docking                             # noqa: E402
from automato_control_service import route_runner as rr                  # noqa: E402
from automato_control_service import harvest_dispatcher as hd            # noqa: E402
from automato_control_service.harvest_dispatcher import (                # noqa: E402
    REASON_DOCK_FAILED,
    STATUS_COMPLETED,
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
# 예냉실 — 수확지(4)에서 되돌아오는 자리(12)에 있다고 둔다.
PRECOOL_POINT = {"task_point_id": "PRECOOL_01", "point_type": "PRECOOL",
                 "waypoint_id": 12, "x": 3.6, "y": 0.0, "yaw": 0.0}


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

    def __init__(self, server_up=True, code=0, codes=None):
        self.server_up = server_up
        self.code = code          # 0 도착 / 1 막힘 / 2 중단
        # 하달마다 결과를 달리 줄 때(마지막 값으로 클램프). FakeDock 과 같은 관례로,
        # '언도킹만 실패' 처럼 첫 하달을 골라 실패시킬 때 쓴다.
        self.codes = list(codes) if codes else None
        self.dispatched = []
        # 하달 배열의 yaw 까지 남긴다 — 언도킹은 '어느 노드로' 만큼 '어느 방향으로'가
        # 중요하다(방향을 안 실으면 0=정동쪽 폴백이 나가 도킹 자리에서 크게 돈다).
        self.yaws = []

    def wait_for_server(self, timeout_sec=None):
        return self.server_up

    def send_goal_async(self, goal, feedback_callback=None):
        wps = [w.waypoint_id for w in goal.waypoints]
        self.dispatched.append(wps)
        self.yaws.append([w.yaw for w in goal.waypoints])
        code = self.code
        if self.codes:
            code = self.codes[min(len(self.dispatched) - 1, len(self.codes) - 1)]
        result_future = Future()

        def drive():
            for wp in wps:
                if feedback_callback is not None:
                    feedback_callback(_Feedback(wp))
            result_future.set_result(_Result(code, wps[-1]))

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
    """Dock 결과를 code 로 지정하는 가짜 클라이언트. calls 로 하달 횟수를 센다.

    codes 를 주면 호출마다 그 순서대로 result_code 를 돌려준다(마지막 값으로 클램프) —
    '수확지 도킹은 성공, 예냉실 도킹은 실패' 처럼 호출별 결과를 달리 줄 때 쓴다.
    """

    def __init__(self, code=0, server_up=True, codes=None):
        self.code = code                  # 0 성공 / 1 마커 미검출 / 2 정차오차 초과
        self.codes = codes                # 호출별 result_code(마지막 값 클램프). None 이면 code 고정
        self.server_up = server_up
        self.calls = 0

    def wait_for_server(self, timeout_sec=None):
        return self.server_up

    def send_goal_async(self, goal, feedback_callback=None):
        self.calls += 1
        self.goal = goal                  # 테스트가 Goal 내용을 들여다본다
        code = (self.codes[min(self.calls - 1, len(self.codes) - 1)]
                if self.codes is not None else self.code)
        if feedback_callback is not None:
            feedback_callback(_DockFeedback("REVERSING"))
        result_future = Future()
        result_future.set_result(
            _DockResult(code, "ok" if code == 0 else "dock fail"))
        goal_future = Future()
        goal_future.set_result(_Handle(result_future))
        return goal_future


class _HarvestResult:
    def __init__(self, normal, discard, failed, exit_reason):
        self.normal_count = normal
        self.discard_count = discard
        self.failed_count = failed
        self.exit_reason = exit_reason
        self.message = "ok"


class _HarvestResponse:
    """액션 결과 응답 — 본문(result)과 **goal 상태(status)** 를 함께 담는다.

    Harvest 는 result_code 가 없어서 이 status 로 성공/중단을 가린다.
    """
    def __init__(self, status, result):
        self.status = status
        self.result = result


class _HarvestFeedback:
    def __init__(self, round_, normal, discard, failed, remaining):
        self.feedback = self
        self.round = round_
        self.normal_count = normal
        self.discard_count = discard
        self.failed_count = failed
        self.remaining_in_round = remaining


class FakeHarvest:
    """수확 로봇 흉내 — 피드백을 몇 번 보낸 뒤 지정한 상태로 끝난다."""

    def __init__(self, status=GoalStatus.STATUS_SUCCEEDED, normal=5, discard=2,
                 failed=1, exit_reason="FULL", server_up=True, accepted=True,
                 feedbacks=()):
        self.status = status
        self.result = _HarvestResult(normal, discard, failed, exit_reason)
        self.server_up = server_up
        self.accepted = accepted
        self.feedbacks = feedbacks
        self.calls = 0
        self.goal = None

    def wait_for_server(self, timeout_sec=None):
        return self.server_up

    def send_goal_async(self, goal, feedback_callback=None):
        self.calls += 1
        self.goal = goal
        for fb in self.feedbacks:
            if feedback_callback is not None:
                feedback_callback(fb)
        result_future = Future()
        result_future.set_result(_HarvestResponse(self.status, self.result))
        goal_future = Future()
        handle = _Handle(result_future)
        handle.accepted = self.accepted
        goal_future.set_result(handle)
        return goal_future


class _UnloadResult:
    def __init__(self, code, message):
        self.result = self
        self.result_code = code
        self.message = message


class _UnloadFeedback:
    def __init__(self, phase):
        self.feedback = self
        self.phase = phase


class FakeUnload:
    """하역 로봇 흉내 — result_code 로 성공/실패를 지정한다."""

    def __init__(self, code=0, server_up=True):
        self.code = code              # 0 성공 / 1 손잡이 파지 실패 / 2 중단
        self.server_up = server_up
        self.calls = 0
        self.goal = None

    def wait_for_server(self, timeout_sec=None):
        return self.server_up

    def send_goal_async(self, goal, feedback_callback=None):
        self.calls += 1
        self.goal = goal
        if feedback_callback is not None:
            feedback_callback(_UnloadFeedback("SHAKE"))
        result_future = Future()
        result_future.set_result(
            _UnloadResult(self.code, "ok" if self.code == 0 else "unload fail"))
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
    monkeypatch.setattr(hd, "HEARTBEAT_SEC", 0.02)   # 수확 결과 대기


def _make(nav=None, dock=None, harvest=None):
    engine = RoutingEngine(WAYPOINTS, CORRIDORS, reservation_ttl=60.0)
    log = _Log()
    disp = HarvestDispatcher(log)
    disp.runner.wp_meta = dict(WP_META)
    return (engine, disp, log,
            (nav if nav is not None else FakeNav()),
            (dock if dock is not None else FakeDock()),
            (harvest if harvest is not None else FakeHarvest()))


def _clients(nav, dock=None, harvest=None):
    """E2~E4 가 쓰는 것은 nav·dock·harvest — unload(E6)는 아직 부르지 않는다
    (None 이라 실수로 부르면 즉시 AttributeError 로 드러난다).

    dock_for(method) 는 방식과 무관하게 이 fake dock 을 준다(수확지·예냉실 모두 floor).
    """
    return {"nav": nav, "harvest": harvest, "unload": None,
            "dock_for": lambda method: dock}


def _no_reservations(engine):
    snap = engine.reservation_snapshot()
    return snap["corridors"] == {} and snap["nodes"] == {}


def _only_standing_node(engine, node_id, robot_id="dg_01"):
    """성공 종료 뒤 남아야 할 예약 = '로봇이 서 있는 자리' 하나뿐인가.

    수확이 성공하면 마지막 자리를 일부러 남긴다 — 충전소 복귀(_return_and_dock)가
    같은 로봇 자격으로 이어받아 도킹 성공 시점에 해제하기 때문이다. 여기서 놓아
    버리면 그 찰나에 남이 예냉실 자리로 들어와 복귀 출발선이 막힌다.
    (실패로 끝났을 때는 뒷단계가 없으므로 _no_reservations 가 맞다.)
    """
    snap = engine.reservation_snapshot()
    return snap["corridors"] == {} and snap["nodes"] == {node_id: robot_id}


# ------------------------------- 언도킹(도킹 탈출) ------------------------------- #
def test_충전소에서_언도킹부터_하달한다(fast_timing):
    """로봇은 충전기에 '물리적으로 붙어' 있다 — 첫 하달은 진입 노드 한 개짜리여야 한다.

    이게 없으면 Nav2 가 좁은 충전 공간에서 회전부터 시작해 충전 단자를 긁는다.
    순찰(_lead_in ①)이 같은 이유로 하는 일을 수확도 해야 한다.
    """
    engine, disp, _log, nav, dock, harv = _make()

    disp.run_harvest(1, "dg_01", HARVEST_POINT, engine,
                     _clients(nav, dock, harv), start_wp=15)

    assert nav.dispatched[0] == [15], \
        f"첫 하달이 언도킹([15] 단독)이 아니다: {nav.dispatched}"


def test_언도킹은_도킹을_마친_방향으로_하달한다(fast_timing):
    """언도킹 하달에는 '지금 보고 있는 방향'을 실어야 한다.

    방향을 안 실으면 route_runner 가 '다음 노드 쪽'을 계산하려다 노드가 하나뿐이라
    0.0(정동쪽)으로 폴백한다. 로봇은 이동거리가 10cm 미만이면 그 값을 그대로 쓰므로
    (navigate_server 의 min_travel_m), 벽에 붙은 자리에서 엉뚱한 방향으로 크게 돈다.
    실을 값은 진입 노드 yaw + π — 후진 도킹이라 로봇은 진입 방향의 반대를 본다.
    """
    engine, disp, _log, nav, dock, harv = _make()

    disp.run_harvest(1, "dg_01", HARVEST_POINT, engine,
                     _clients(nav, dock, harv), start_wp=15)

    # 테스트 그래프의 진입 노드 yaw 는 0.0 → 도킹을 마친 방향은 ±π 다.
    undock_yaw = nav.yaws[0][0]
    assert abs(abs(undock_yaw) - math.pi) < 1e-6, \
        f"언도킹 방향이 도킹 자세(±π)가 아니다: {undock_yaw} (0.0 이면 폴백이 나간 것)"


def test_언도킹이_실패하면_주행하지_않고_FAILED(fast_timing):
    """충전기에서 못 빠져나왔는데 다음 목표를 하달하면 그 자리에서 회전한다."""
    engine, disp, log, _nav, dock, harv = _make()
    nav = FakeNav(codes=[1])          # 첫 하달(언도킹)만 실패시킨다

    status, _reason, _last = disp.run_harvest(
        1, "dg_01", HARVEST_POINT, engine, _clients(nav, dock, harv), start_wp=15)

    assert status == STATUS_FAILED
    assert nav.dispatched == [[15]], \
        f"언도킹이 실패했는데 주행을 계속했다: {nav.dispatched}"
    assert log.has("E2 언도킹 실패"), f"실패 로그가 없다: {log.lines}"
    assert _no_reservations(engine), "언도킹 실패 후 예약이 남았다"


# --------------------------------------------------------------------------- #
def test_수확지까지_주행하고_자리를_반납한다(fast_timing):
    """1·2 — 수확지에 도달하고, 끝나면 예약이 하나도 안 남는다."""
    engine, disp, log, nav, dock, harv = _make()

    status, _reason, _last = disp.run_harvest(
        1, "dg_01", HARVEST_POINT, engine, _clients(nav, dock, harv), start_wp=15)

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
    engine, disp, _log, nav, dock, harv = _make()

    disp.run_harvest(1, "dg_01", HARVEST_POINT, engine,
                     _clients(nav, dock, harv), start_wp=15)

    flat = [wp for seg in nav.dispatched for wp in seg]
    assert set(flat) <= set(WAYPOINTS), \
        f"그래프에 없는 노드(짝 등)가 하달됐다: {flat}"


def test_경로가_없으면_FAILED_이고_예약도_안_남는다(fast_timing):
    """3 — 남이 길목을 다 쥐고 있어 갈 수 없으면 끊는다(그때도 뒷정리는 한다)."""
    engine, disp, log, nav, dock, harv = _make()
    # 유일한 통로(15-12)의 도착 자리를 남이 쥐고 있으면 이 일직선 그래프에선 우회로가 없다.
    assert engine.try_reserve(engine.node_slot(12), "dg_09") is True

    status, _reason, _last = disp.run_harvest(
        1, "dg_01", HARVEST_POINT, engine, _clients(nav, dock, harv), start_wp=15)

    assert status == STATUS_FAILED
    assert log.has("E2 주행 실패"), f"실패 로그가 없다: {log.lines}"
    assert engine.reservation_snapshot()["nodes"] == {12: "dg_09"}, \
        "남의 예약을 건드렸거나 내 예약이 남았다"
    assert engine.reservation_snapshot()["corridors"] == {}, "통로 예약이 누수됐다"


def test_로봇이_중단을_보고하면_FAILED(fast_timing):
    """3 — 로봇이 스스로 멈췄다(result_code=2)면 재시도 없이 끊는다."""
    engine, disp, _log, nav, dock, harv = _make(FakeNav(code=2))

    status, _reason, _last = disp.run_harvest(
        1, "dg_01", HARVEST_POINT, engine, _clients(nav, dock, harv), start_wp=15)

    assert status == STATUS_FAILED
    assert _no_reservations(engine), "중단 보고 뒤에도 예약이 남았다"


def test_출발점을_모르면_나서지도_않는다(fast_timing):
    """4 — 순찰과 달리 폴백하지 않는다. 한 발짝도 움직이면 안 된다."""
    engine, disp, log, nav, dock, harv = _make()

    status, _reason, _last = disp.run_harvest(
        1, "dg_01", HARVEST_POINT, engine, _clients(nav, dock, harv), start_wp=None)

    assert status == STATUS_FAILED
    assert nav.dispatched == [], "출발점을 모르는데 로봇을 움직였다"
    assert log.has("출발 노드"), f"원인이 로그에 안 남았다: {log.lines}"
    assert _no_reservations(engine)


def test_그래프에_없는_수확지는_거절한다(fast_timing):
    """DB 에는 있지만 라우팅 그래프에 없는 지점 — 경로 계산이 불가능하다."""
    engine, disp, _log, nav, dock, harv = _make()
    ghost = dict(HARVEST_POINT, waypoint_id=999)

    status, _reason, _last = disp.run_harvest(
        1, "dg_01", ghost, engine, _clients(nav, dock, harv), start_wp=15)

    assert status == STATUS_FAILED
    assert nav.dispatched == []


def test_액션서버가_없으면_즉시_끊는다(fast_timing):
    """안 걸러내면 Goal 마다 수락 타임아웃을 다 기다린 뒤에야 실패한다."""
    engine, disp, _log, nav, dock, harv = _make(FakeNav(server_up=False))

    status, _reason, _last = disp.run_harvest(
        1, "dg_01", HARVEST_POINT, engine, _clients(nav, dock, harv), start_wp=15)

    assert status == STATUS_FAILED
    assert nav.dispatched == []


# ------------------------------ E2 ChArUco 도킹 ------------------------------ #
def test_도착하면_수확지_마커로_도킹한다(fast_timing):
    """5 — 도착 후 도킹을 한 번 하달하고, Goal 에 그 지점 id 가 실린다(floor 는 마커리스)."""
    engine, disp, log, nav, dock, harv = _make()

    disp.run_harvest(1, "dg_01", HARVEST_POINT, engine,
                     _clients(nav, dock, harv), start_wp=15)

    assert dock.calls == 1, f"도킹 하달 횟수가 이상하다: {dock.calls}"
    # 충전소가 아니라 '이 수확지'로 도킹해야 한다 — 지점 id 를 잘못 실으면 로봇이
    # 엉뚱한 도크를 찾는다. (floor 도킹은 마커리스라 Goal 에 마커 규격이 없다)
    assert dock.goal.task_point_id == "HARVEST_01"
    assert log.has("E2 도킹 완료"), f"도킹 완료 로그가 없다: {log.lines}"


def test_도킹_중에는_자리를_쥐고_있다(fast_timing):
    """도킹은 수십 초 걸린다 — 그동안 자리를 놓으면 남이 그 지점으로 들어온다.

    도킹 시점에 예약표를 들여다봐, 로봇이 수확지 자리를 쥔 채인지 확인한다.
    (끝난 뒤가 아니라 '도킹 중'이어야 의미가 있다 — 끝나면 finally 가 반납한다.)
    """
    engine, disp, _log, nav, dock, harv = _make()
    seen = {}

    real_send = dock.send_goal_async

    def spy(goal, feedback_callback=None):
        seen["holder"] = engine.holder_of(engine.node_slot(4))
        return real_send(goal, feedback_callback)

    dock.send_goal_async = spy
    disp.run_harvest(1, "dg_01", HARVEST_POINT, engine,
                     _clients(nav, dock, harv), start_wp=15)

    assert seen.get("holder") == "dg_01", \
        f"도킹 중에 수확지 자리를 안 쥐고 있었다(홀더={seen.get('holder')})"


def test_도킹이_계속_실패하면_N회_재시도_후_DOCK_FAILED(fast_timing):
    """N_dock 소진 → FAILED + DOCK_FAILED 사유(노드가 이걸 보고 관리자에게 알린다)."""
    engine, disp, log, nav, dock, harv = _make(dock=FakeDock(code=1))

    status, reason, _last = disp.run_harvest(
        1, "dg_01", HARVEST_POINT, engine, _clients(nav, dock, harv), start_wp=15)

    assert dock.calls == docking.DOCK_RETRY_MAX, \
        f"재시도 횟수가 규격(N_dock)과 다르다: {dock.calls}"
    assert status == STATUS_FAILED
    assert reason == REASON_DOCK_FAILED
    assert log.has("E2 도킹 실패"), f"실패 로그가 없다: {log.lines}"
    assert _no_reservations(engine), "도킹 실패 뒤에도 예약이 남았다"


def test_주행_실패는_도킹까지_가지_않는다(fast_timing):
    """길이 막혀 못 갔으면 도킹 단계는 아예 없다(그리고 DOCK_FAILED 도 아니다)."""
    engine, disp, _log, nav, dock, harv = _make()
    assert engine.try_reserve(engine.node_slot(12), "dg_09") is True

    status, reason, _last = disp.run_harvest(
        1, "dg_01", HARVEST_POINT, engine, _clients(nav, dock, harv), start_wp=15)

    assert dock.calls == 0, "도착도 못 했는데 도킹을 시도했다"
    assert status == STATUS_FAILED
    assert reason is None, "주행 실패에 도킹 실패 사유가 붙었다"


# ------------------------------ E3~E4 수확 ------------------------------ #
def test_도킹_후_수확을_시킨다(fast_timing):
    """6 — 도킹이 끝나면 Harvest 를 한 번 하달하고, 만차 기준이 Goal 에 실린다."""
    engine, disp, log, nav, dock, harv = _make()

    disp.run_harvest(1, "dg_01", HARVEST_POINT, engine,
                     _clients(nav, dock, harv), start_wp=15)

    assert harv.calls == 1, f"수확 하달 횟수가 이상하다: {harv.calls}"
    assert harv.goal.max_capacity == hd.HARVEST_MAX_CAPACITY
    assert log.has("E3~4 수확 종료"), f"수확 종료 로그가 없다: {log.lines}"
    assert _no_reservations(engine), "수확이 끝났는데 예약이 남았다"


def test_중단된_수확을_성공으로_오인하지_않는다(fast_timing):
    """가장 중요한 판정 — Harvest 에는 result_code 가 없다.

    로봇이 중간에 뻗으면(ABORTED) 집계 필드는 0 으로 채워져 오는데, 본문만 읽으면
    '아무것도 못 땄지만 정상 종료'와 구분되지 않는다. 그러면 빈 바구니를 예냉실까지
    나르게 된다. goal 상태를 봐야만 걸러진다.
    """
    engine, disp, log, nav, dock, harv = _make(
        harvest=FakeHarvest(status=GoalStatus.STATUS_ABORTED,
                            normal=0, discard=0, failed=0, exit_reason=""))

    status, reason, _last = disp.run_harvest(
        1, "dg_01", HARVEST_POINT, engine,
        _clients(nav, dock, harv), start_wp=15)

    assert status == STATUS_FAILED
    assert reason is None, "수확 중단은 도킹 실패가 아니다"
    assert not log.has("E3~4 수확 종료"), \
        "중단(ABORTED)인데 수확이 정상 종료된 것으로 처리됐다"
    assert _no_reservations(engine), "수확 중단 뒤에도 예약이 남았다"


def test_진행_보고를_콜백으로_넘긴다(fast_timing):
    """수확 중 들어오는 보고를 그대로 호출부(노드)에 넘긴다 — 발송은 노드가 한다."""
    engine, disp, _log, nav, dock, harv = _make(
        harvest=FakeHarvest(feedbacks=[
            _HarvestFeedback(1, 2, 0, 0, 3),
            _HarvestFeedback(2, 5, 2, 1, 0),
        ]))
    got = []

    disp.run_harvest(1, "dg_01", HARVEST_POINT, engine,
                     _clients(nav, dock, harv), start_wp=15,
                     on_progress=got.append)

    assert len(got) == 2, f"진행 보고가 그대로 안 넘어왔다: {got}"
    assert got[0]["round"] == 1 and got[0]["normal_count"] == 2
    assert got[1]["remaining_in_round"] == 0


def test_수확_중에도_자리를_쥐고_있다(fast_timing):
    """수확은 분 단위다 — 그동안 자리를 놓으면 팔 작업 중에 남이 들어온다."""
    engine, disp, _log, nav, dock, harv = _make()
    seen = {}
    real_send = harv.send_goal_async

    def spy(goal, feedback_callback=None):
        seen["holder"] = engine.holder_of(engine.node_slot(4))
        return real_send(goal, feedback_callback)

    harv.send_goal_async = spy
    disp.run_harvest(1, "dg_01", HARVEST_POINT, engine,
                     _clients(nav, dock, harv), start_wp=15)

    assert seen.get("holder") == "dg_01", \
        f"수확 중에 자리를 안 쥐고 있었다(홀더={seen.get('holder')})"


def test_수확_서버가_없거나_거부하면_FAILED(fast_timing):
    """DG 는 '도킹 성공한 task' 의 goal 만 받는다 → 거부는 상태 불일치 신호다."""
    engine, disp, _log, nav, dock, harv = _make(
        harvest=FakeHarvest(accepted=False))

    status, _reason, _last = disp.run_harvest(
        1, "dg_01", HARVEST_POINT, engine,
        _clients(nav, dock, harv), start_wp=15)

    assert status == STATUS_FAILED
    assert _no_reservations(engine)


def test_도킹_실패하면_수확까지_가지_않는다(fast_timing):
    """도킹이 안 됐는데 팔을 뻗으면 엉뚱한 곳을 집는다."""
    engine, disp, _log, nav, dock, harv = _make(dock=FakeDock(code=1))

    status, reason, _last = disp.run_harvest(
        1, "dg_01", HARVEST_POINT, engine,
        _clients(nav, dock, harv), start_wp=15)

    assert harv.calls == 0, "도킹에 실패했는데 수확을 시켰다"
    assert status == STATUS_FAILED
    assert reason == REASON_DOCK_FAILED


# ------------------------ E5 실적 저장 + 예냉실 이송 ------------------------ #
def _e5(disp, engine, clients, save_batch=None, precool=PRECOOL_POINT):
    """E5 까지 가는 호출(예냉실 정보와 저장 콜백을 함께 넘긴다)."""
    return disp.run_harvest(
        1, "dg_01", HARVEST_POINT, engine, clients, start_wp=15,
        precool_point=precool, save_batch=save_batch)


def test_실적을_이송_전에_적는다(fast_timing):
    """E5 의 핵심 순서 — 예냉실로 출발하기 전에 장부를 남긴다.

    이송은 분 단위라 그 사이 로봇·프로세스가 죽을 수 있다. 그때 아직 안 적었으면
    애써 딴 실적이 통째로 사라진다(바구니엔 있는데 시스템은 모르는 상태).
    """
    engine, disp, log, nav, dock, harv = _make()
    seen = {}

    def save(harvested):
        # 저장이 불린 시점에 '아직 예냉실로 출발하지 않았어야' 한다.
        seen["dispatch_count"] = len(nav.dispatched)
        seen["harvested"] = harvested
        return 77

    _e5(disp, engine, _clients(nav, dock, harv), save_batch=save)

    assert "harvested" in seen, "실적 저장 콜백이 불리지 않았다"
    assert seen["harvested"]["normal_count"] == 5
    # 수확지까지의 주행 하달 횟수 그대로여야 한다(예냉실 하달이 아직 없다).
    after = len(nav.dispatched)
    assert seen["dispatch_count"] < after, \
        "예냉실로 출발한 뒤에 실적을 적었다 — 이송 중 사고나면 실적이 사라진다"
    assert log.has("batch_id=77"), f"batch_id 가 로그에 안 남았다: {log.lines}"


def test_예냉실로_출발하기_전에도_언도킹한다(fast_timing):
    """수확지도 도킹 상태다 — H 마커 도킹은 후면이 벽에서 3cm(floor_fsm 의
    WALL_GAP_TARGET). 바로 예냉실 쪽으로 방향을 틀면 후면 코너가 벽을 긁는다.
    충전소 출발과 같은 이유로 여기서도 한 스텝 빼낸 뒤 이송을 시작해야 한다.
    """
    engine, disp, _log, nav, dock, harv = _make()

    _e5(disp, engine, _clients(nav, dock, harv), save_batch=lambda h: 1)

    # 수확지(4) 한 개짜리 하달 = 언도킹. 주행 세그먼트는 여러 노드를 묶어 나간다.
    assert [4] in nav.dispatched, \
        f"수확지 언도킹([4] 단독) 하달이 없다: {nav.dispatched}"
    undock_at = nav.dispatched.index([4])
    # 순서: 언도킹이 '예냉실로 떠나는 하달'보다 앞에 있어야 의미가 있다.
    assert nav.dispatched[-1][-1] == 12, f"예냉실(12)까지 못 갔다: {nav.dispatched}"
    assert undock_at < len(nav.dispatched) - 1, \
        f"언도킹이 이송 하달보다 뒤에 있다: {nav.dispatched}"
    # 충전소와 같은 규칙 — 도킹을 마친 방향(진입 노드 yaw + π)을 실어야 한다.
    assert abs(abs(nav.yaws[undock_at][0]) - math.pi) < 1e-6, \
        f"언도킹 방향이 도킹 자세가 아니다: {nav.yaws[undock_at]}"


def test_예냉실까지_이송하고_도킹한다(fast_timing):
    """수확지 → 예냉실 이동 후 도킹. 도킹은 수확지·예냉실 두 번 일어난다."""
    engine, disp, log, nav, dock, harv = _make()

    _e5(disp, engine, _clients(nav, dock, harv), save_batch=lambda h: 1)

    assert nav.dispatched[-1][-1] == 12, \
        f"예냉실(12)까지 못 갔다: {nav.dispatched}"
    assert dock.calls == 2, f"도킹이 두 번(수확지·예냉실) 일어나야 한다: {dock.calls}"
    assert dock.goal.task_point_id == "PRECOOL_01", "마지막 도킹이 예냉실이 아니다"
    assert log.has("E5 예냉실 도킹 완료"), f"완료 로그가 없다: {log.lines}"
    # 예냉실 자리 하나만 남는다 — 충전소 복귀가 이어받아 도킹 후 해제한다.
    assert _only_standing_node(engine, 12), \
        f"예냉실 자리만 남아야 한다(복귀에 인계): {engine.reservation_snapshot()}"


def test_실적_저장이_실패해도_이송은_계속한다(fast_timing):
    """토마토는 이미 바구니에 있고 딴 순간부터 상한다 — 기록 문제로 냉장을 미루면
    실물을 버린다. 되돌릴 수 없는 쪽은 실물이다."""
    engine, disp, log, nav, dock, harv = _make()

    def save_broken(_harvested):
        raise RuntimeError("DB 연결 끊김")

    _e5(disp, engine, _clients(nav, dock, harv), save_batch=save_broken)

    assert nav.dispatched[-1][-1] == 12, "저장 실패로 이송까지 멈췄다"
    assert log.has("수확 실적 저장 실패"), f"실패가 로그에 안 남았다: {log.lines}"


def test_예냉실이_그래프에_없으면_이송_실패(fast_timing):
    """갈 곳이 없으면 이송은 못 하지만, 실적은 이미 저장돼 있어야 한다."""
    engine, disp, log, nav, dock, harv = _make()
    saved = []
    ghost = dict(PRECOOL_POINT, waypoint_id=999)

    status, _reason, _last = _e5(disp, engine, _clients(nav, dock, harv),
                                 save_batch=lambda h: saved.append(h) or 5,
                                 precool=ghost)

    assert status == STATUS_FAILED
    assert saved, "이송이 불가능해도 수확 실적은 남아야 한다"
    assert _no_reservations(engine)


def test_예냉실_도킹_실패는_DOCK_FAILED(fast_timing):
    """수확지 도킹은 성공하고 예냉실 도킹만 실패 → 관리자 통지(DOCK_FAILED)가 나가야 한다."""
    # codes=[0, 1]: 1번째(수확지) 성공, 2번째부터(예냉실 + 재시도) 실패.
    engine, disp, _log, nav, dock, harv = _make(dock=FakeDock(codes=[0, 1]))

    status, reason, _last = _e5(disp, engine, _clients(nav, dock, harv),
                                save_batch=lambda h: 3)

    assert dock.calls >= 2, "수확지(성공)·예냉실(실패) 도킹이 모두 일어나야 한다"
    assert status == STATUS_FAILED
    assert reason == REASON_DOCK_FAILED
    assert _no_reservations(engine), "도킹 실패 뒤에도 예약이 남았다"


def test_수확이_실패하면_이송도_저장도_없다(fast_timing):
    """못 땄으면 적을 것도 나를 것도 없다."""
    engine, disp, _log, nav, dock, harv = _make(
        harvest=FakeHarvest(status=GoalStatus.STATUS_ABORTED))
    saved = []

    _e5(disp, engine, _clients(nav, dock, harv),
        save_batch=lambda h: saved.append(h) or 1)

    assert saved == [], "수확이 중단됐는데 실적을 적었다"
    assert nav.dispatched[-1][-1] == 4, "수확 실패인데 예냉실로 갔다"


# ------------------------ E6 하역 + 완료 통지 ------------------------ #
def _e6(disp, engine, nav, dock, harv, unload, **kw):
    """E6 까지 가는 호출(하역 클라이언트와 콜백들을 함께 넘긴다)."""
    clients = {"nav": nav, "harvest": harv, "unload": unload,
               "dock_for": lambda method: dock}
    return disp.run_harvest(
        1, "dg_01", HARVEST_POINT, engine, clients, start_wp=15,
        precool_point=PRECOOL_POINT,
        save_batch=kw.get("save_batch", lambda h: 42),
        save_unload=kw.get("save_unload"),
        on_completed=kw.get("on_completed"))


def test_성공하면_복귀_출발점을_돌려준다(fast_timing):
    """3-tuple 의 세 번째 값 — 노드가 이걸 받아 충전소 복귀(_return_and_dock)를 잇는다.

    None 이 오면 복귀가 통째로 생략돼 로봇이 예냉실에 도킹된 채 남는다. 그러면 다음
    task 는 '이 로봇은 자기 충전소에 있다'(_start_waypoint_for)고 가정하므로, 실제
    위치와 어긋난 출발점으로 경로를 예약한다.
    """
    engine, disp, _log, nav, dock, harv = _make()

    status, _reason, last_wp = _e6(disp, engine, nav, dock, harv, FakeUnload())

    assert status == STATUS_COMPLETED
    assert last_wp == 12, f"복귀 출발점(예냉실 12)을 안 돌려줬다: {last_wp}"


def test_실패하면_복귀_출발점을_돌려주지_않는다(fast_timing):
    """실패는 복귀하지 않는다 — 자리도 디스패처가 그 자리에서 반납한다."""
    engine, disp, _log, _nav, dock, harv = _make()
    nav = FakeNav(codes=[1])          # 언도킹부터 실패시킨다

    status, _reason, last_wp = disp.run_harvest(
        1, "dg_01", HARVEST_POINT, engine, _clients(nav, dock, harv),
        start_wp=15, precool_point=PRECOOL_POINT)

    assert status == STATUS_FAILED
    assert last_wp is None, f"실패인데 복귀 출발점을 넘겼다: {last_wp}"
    assert _no_reservations(engine), "실패로 끝났는데 자리가 남았다"


def test_하역하고_입고를_기록하고_성공으로_마감한다(fast_timing):
    """정상 흐름의 끝 — 여기서 처음으로 COMPLETED 가 나온다."""
    engine, disp, log, nav, dock, harv = _make()
    unload = FakeUnload(code=0)
    logged = []

    status, reason, _last = _e6(disp, engine, nav, dock, harv, unload,
                                save_unload=lambda h: logged.append(h) or 9)

    assert (status, reason) == (STATUS_COMPLETED, None)
    assert unload.calls == 1, f"하역 하달 횟수가 이상하다: {unload.calls}"
    assert unload.goal.shake_delay_sec == hd.UNLOAD_SHAKE_DELAY_SEC
    assert logged and logged[0]["normal_count"] == 5, "입고 기록이 안 남았다"
    assert log.has("E6 하역 완료"), f"하역 완료 로그가 없다: {log.lines}"
    # 수확 task 는 여기서 끝나지만 예약은 하나 남는다 — 로봇이 예냉실에 서 있고,
    # 충전소 복귀(노드가 잇는다)가 그 자리를 이어받기 때문이다.
    assert _only_standing_node(engine, 12), \
        f"예냉실 자리만 남아야 한다(복귀에 인계): {engine.reservation_snapshot()}"


def test_하역이_실패해도_작업은_성공이다(fast_timing):
    """E6 의 핵심 판정 — 하역은 보너스다.

    이 task 의 목적은 '따서 예냉실로 옮기기'이고 도착한 순간 달성됐다. 바구니를
    자동으로 비우는 건 편의 기능이라 실패하면 사람이 손으로 비우면 된다. 여기서
    FAILED 로 되돌리면 '토마토는 무사히 옮겨졌는데 수확은 실패'라는 기록이 남는다.
    """
    engine, disp, log, nav, dock, harv = _make()
    unload = FakeUnload(code=1)          # 손잡이 파지 실패
    logged = []

    status, reason, _last = _e6(disp, engine, nav, dock, harv, unload,
                                save_unload=lambda h: logged.append(h) or 9)

    assert status == STATUS_COMPLETED, "하역 실패로 수확 task 까지 실패로 뒤집혔다"
    assert reason is None
    assert logged == [], "하역이 실패했는데 입고를 기록했다(재고가 실제보다 는다)"
    assert log.has("E6 하역 실패"), f"실패가 로그에 안 남았다: {log.lines}"


def test_완료_통지에_실적과_장부번호가_실린다(fast_timing):
    """관리자 화면이 '무엇을 얼마나 땄는지' 알려면 집계와 batch_id 가 함께 가야 한다."""
    engine, disp, _log, nav, dock, harv = _make()
    got = []

    _e6(disp, engine, nav, dock, harv, FakeUnload(),
        save_batch=lambda h: 123, on_completed=got.append)

    assert len(got) == 1, f"완료 통지가 정확히 한 번 가야 한다: {got}"
    assert got[0]["batch_id"] == 123
    assert got[0]["normal_count"] == 5 and got[0]["discard_count"] == 2
    assert got[0]["exit_reason"] == "FULL"


def test_통지가_실패해도_작업_마감은_막지_않는다(fast_timing):
    """알림은 못 보내도 실적은 DB 에 있다 — 통지 때문에 성공을 실패로 만들면 안 된다."""
    engine, disp, _log, nav, dock, harv = _make()

    def broken(_summary):
        raise RuntimeError("웹 서비스 다운")

    status, _reason, _last = _e6(disp, engine, nav, dock, harv, FakeUnload(),
                                 on_completed=broken)

    assert status == STATUS_COMPLETED


def test_하역_서버가_없어도_작업은_성공이다(fast_timing):
    """하역 서버가 안 떠 있어도 이송은 끝났다 — 사람이 비우면 된다."""
    engine, disp, _log, nav, dock, harv = _make()
    logged = []

    status, _reason, _last = _e6(disp, engine, nav, dock, harv,
                                 FakeUnload(server_up=False),
                                 save_unload=lambda h: logged.append(h) or 9)

    assert status == STATUS_COMPLETED
    assert logged == [], "하역을 못 했는데 입고를 기록했다"


def test_입고_기록이_실패해도_작업은_성공이다(fast_timing):
    """하역은 실제로 됐다 — 기록 실패로 성공을 뒤집으면 실물과 어긋난다."""
    engine, disp, log, nav, dock, harv = _make()

    def broken(_h):
        raise RuntimeError("DB 연결 끊김")

    status, _reason, _last = _e6(disp, engine, nav, dock, harv, FakeUnload(),
                                 save_unload=broken)

    assert status == STATUS_COMPLETED
    assert log.has("입고 기록 실패"), f"실패가 로그에 안 남았다: {log.lines}"


def test_예냉실에_못_가면_하역도_완료통지도_없다(fast_timing):
    """도착을 못 했으면 비울 것도, 알릴 것도 없다."""
    engine, disp, _log, nav, dock, harv = _make()
    unload = FakeUnload()
    got = []
    clients = {"nav": nav, "harvest": harv, "unload": unload,
               "dock_for": lambda method: dock}

    status, _reason, _last = disp.run_harvest(
        1, "dg_01", HARVEST_POINT, engine, clients, start_wp=15,
        precool_point=dict(PRECOOL_POINT, waypoint_id=999),
        save_batch=lambda h: 1, on_completed=got.append)

    assert status == STATUS_FAILED
    assert unload.calls == 0 and got == []
