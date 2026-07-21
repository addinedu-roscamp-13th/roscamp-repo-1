#!/usr/bin/env python3
"""RP-78 ④ 라우팅/예약 엔진 단위테스트 — 그래프만으로 검증(ROS/DB 불필요).

DoD: "routing/예약 엔진이 독립 모듈로 구현되고 그래프 단위 테스트 통과".

테스트 그래프 (사각 순환 + 대각선 없음):
    1 --10-- 2
    |        |
   13        11
    |        |
    4 --12-- 3
  통로: 10:(1,2) 11:(2,3) 12:(3,4) 13:(1,4)

실행:
  PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pytest test/test_routing_engine.py -v
"""
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from automato_control_service.routing_engine import RoutingEngine, Route  # noqa: E402

WAYPOINTS = [1, 2, 3, 4]
CORRIDORS = [
    {"corridor_id": 10, "a": 1, "b": 2},
    {"corridor_id": 11, "a": 2, "b": 3},
    {"corridor_id": 12, "a": 3, "b": 4},
    {"corridor_id": 13, "a": 1, "b": 4},
]


class FakeClock:
    """단조 증가 가짜 시계(TTL 만료 테스트용)."""
    def __init__(self, t=0.0):
        self.t = t

    def __call__(self):
        return self.t

    def advance(self, dt):
        self.t += dt


def _engine(ttl=15.0, clock=None):
    return RoutingEngine(WAYPOINTS, CORRIDORS,
                         reservation_ttl=ttl,
                         time_fn=(clock or (lambda: 0.0)))


# ------------------------------ 그래프 조회 ------------------------------ #
def test_corridor_between_direct_neighbors():
    e = _engine()
    assert e.corridor_between(1, 2) == 10
    assert e.corridor_between(2, 1) == 10        # 무방향
    assert e.corridor_between(1, 3) is None      # 직접 인접 아님


def test_find_path_trivial_same_node():
    e = _engine()
    r = e.find_path(2, 2)
    assert isinstance(r, Route)
    assert r.is_trivial
    assert r.hops() == []


def test_find_path_direct():
    e = _engine()
    r = e.find_path(1, 2)
    assert r.nodes == (1, 2)
    assert r.corridors == (10,)
    assert r.hops() == [(2, 10)]


def test_find_path_shortest_two_hops():
    e = _engine()
    r = e.find_path(1, 3)          # 1-2-3 또는 1-4-3 (둘 다 2 hop)
    assert len(r.nodes) == 3
    assert len(r.corridors) == 2
    assert r.nodes[0] == 1 and r.nodes[-1] == 3


def test_find_path_detour_when_corridor_blocked():
    e = _engine()
    # 1->2 직행 통로(10) 차단 → 1-4-3-2 로 우회
    r = e.find_path(1, 2, blocked={10})
    assert r.nodes[0] == 1 and r.nodes[-1] == 2
    assert 10 not in r.corridors
    assert r.nodes == (1, 4, 3, 2)
    assert r.corridors == (13, 12, 11)


def test_find_path_none_when_fully_blocked():
    e = _engine()
    # 2로 들어가는 두 통로(10, 11)를 모두 막으면 도달 불가
    assert e.find_path(1, 2, blocked={10, 11}) is None


def test_find_path_none_unknown_node():
    e = _engine()
    assert e.find_path(1, 999) is None


def test_find_path_prefers_shorter_distance_over_fewer_hops():
    """Dijkstra 핵심: 홉 수가 많아도 '누적 거리'가 짧은 길을 고른다.

    1→4 직행(통로20)은 1홉이지만 길이 10, 우회 1-2-3-4는 3홉이지만 길이 3.
    옛 BFS(홉 최소)라면 직행을 골랐겠지만, length 비용을 쓰는 Dijkstra는 우회를 고른다.
    """
    wps = [1, 2, 3, 4]
    cors = [
        {"corridor_id": 20, "a": 1, "b": 4, "length": 10.0},  # 직행: 1홉, 멀다
        {"corridor_id": 21, "a": 1, "b": 2, "length": 1.0},
        {"corridor_id": 22, "a": 2, "b": 3, "length": 1.0},
        {"corridor_id": 23, "a": 3, "b": 4, "length": 1.0},   # 우회: 3홉, 짧다
    ]
    e = RoutingEngine(wps, cors)
    r = e.find_path(1, 4)
    assert r.nodes == (1, 2, 3, 4)        # 홉 많아도 거리 짧은 우회를 선택
    assert r.corridors == (21, 22, 23)
    assert 20 not in r.corridors          # 길이 10짜리 직행은 버림


# ------------------------------ 통로 예약 ------------------------------ #
def test_reserve_and_block_second_robot():
    e = _engine()
    assert e.try_reserve(10, "dg_01") is True
    # 안전 속성: 같은 통로를 두 로봇에 동시에 허락하지 않는다
    assert e.try_reserve(10, "dg_02") is False
    assert e.holder_of(10) == "dg_01"


def test_reserve_same_robot_is_idempotent():
    e = _engine()
    assert e.try_reserve(10, "dg_01") is True
    assert e.try_reserve(10, "dg_01") is True    # 재예약(하트비트) 허용


def test_release_only_by_holder():
    e = _engine()
    e.try_reserve(10, "dg_01")
    assert e.release(10, "dg_02") is False       # 남이 해제 못 함
    assert e.holder_of(10) == "dg_01"
    assert e.release(10, "dg_01") is True         # 보유자는 해제
    assert e.holder_of(10) is None
    # 해제 후 다른 로봇이 잡을 수 있음
    assert e.try_reserve(10, "dg_02") is True


def test_reserved_corridors_excludes_own():
    e = _engine()
    e.try_reserve(10, "dg_01")
    e.try_reserve(11, "dg_02")
    assert e.reserved_corridors() == {10, 11}
    assert e.reserved_corridors(exclude_robot="dg_01") == {11}


# ------------------------------ TTL / 하트비트 ------------------------------ #
def test_stale_reservation_reclaimed_after_ttl():
    clk = FakeClock()
    e = _engine(ttl=15.0, clock=clk)
    assert e.try_reserve(10, "dg_01") is True
    clk.advance(20.0)                             # TTL(15) 초과 → 죽은 예약
    # 하트비트가 끊긴 통로는 다른 로봇이 회수 가능
    assert e.try_reserve(10, "dg_02") is True
    assert e.holder_of(10) == "dg_02"


def test_heartbeat_keeps_reservation_alive():
    clk = FakeClock()
    e = _engine(ttl=15.0, clock=clk)
    e.try_reserve(10, "dg_01")
    clk.advance(10.0)
    assert e.heartbeat(10, "dg_01") is True       # 갱신
    clk.advance(10.0)                             # 마지막 하트비트 기준 10초 → 아직 유효
    assert e.try_reserve(10, "dg_02") is False    # 여전히 dg_01 보유


def test_reap_expired_releases_dead_holds():
    clk = FakeClock()
    e = _engine(ttl=15.0, clock=clk)
    e.try_reserve(10, "dg_01")
    e.try_reserve(11, "dg_02")
    clk.advance(20.0)
    reaped = e.reap_expired()
    assert set(reaped) == {10, 11}
    assert e.holder_of(10) is None and e.holder_of(11) is None


# ------------------------------ 데드락(대기 사이클) 회피 ------------------------------ #
def test_would_deadlock_two_robot_cycle():
    # A가 10을 쥐고 11을 기다리고, B가 11을 쥐고 10을 기다리면 서로 물림 → 사이클.
    e = _engine()
    e.try_reserve(10, "dg_01")          # A 보유 10
    e.try_reserve(11, "dg_02")          # B 보유 11
    e.begin_wait("dg_02", 10)           # B는 10(=A 보유)을 기다리는 중
    # 이제 A가 11(=B 보유)을 기다리려 함 → A→B→A 사이클
    assert e.would_deadlock("dg_01", 11) is True


def test_would_deadlock_safe_when_holder_not_waiting():
    # B가 11을 쥐었지만 아무것도 안 기다림 → A가 11을 기다려도 언젠가 풀린다.
    e = _engine()
    e.try_reserve(10, "dg_01")
    e.try_reserve(11, "dg_02")          # B는 대기 안 함
    assert e.would_deadlock("dg_01", 11) is False


def test_would_deadlock_free_corridor_is_safe():
    e = _engine()
    e.try_reserve(10, "dg_01")
    assert e.would_deadlock("dg_01", 12) is False   # 12는 아무도 안 쥠


def test_would_deadlock_three_robot_cycle():
    # A→(11)B→(12)C→(10)A 로 원을 이룸.
    e = _engine()
    e.try_reserve(10, "dg_01"); e.begin_wait("dg_01", 11)
    e.try_reserve(11, "dg_02"); e.begin_wait("dg_02", 12)
    e.try_reserve(12, "dg_03")          # C가 10을 기다리려 하면 원이 닫힌다
    assert e.would_deadlock("dg_03", 10) is True


def test_would_deadlock_three_robot_no_cycle():
    # 사슬: 10→A(11대기)→B. 그런데 B는 안 기다림 → 안전.
    e = _engine()
    e.try_reserve(10, "dg_01"); e.begin_wait("dg_01", 11)
    e.try_reserve(11, "dg_02")          # B는 대기 안 함(사슬이 여기서 끊김)
    assert e.would_deadlock("dg_03", 10) is False


def test_end_wait_breaks_cycle():
    # 사이클 상황을 만든 뒤, 한 로봇이 대기를 풀면 사이클이 사라진다.
    e = _engine()
    e.try_reserve(10, "dg_01")
    e.try_reserve(11, "dg_02")
    e.begin_wait("dg_02", 10)
    assert e.would_deadlock("dg_01", 11) is True
    e.end_wait("dg_02")                 # B가 대기를 포기 → 화살표 제거
    assert e.would_deadlock("dg_01", 11) is False


# ---------------- 확인+획득+대기검사 원자 처리 (reserve_or_wait) ---------------- #
def test_reserve_or_wait_takes_free_corridor():
    e = _engine()
    assert e.reserve_or_wait(10, "dg_01") == "reserved"
    assert e.holder_of(10) == "dg_01"


def test_reserve_or_wait_own_is_reserved():
    e = _engine()
    e.try_reserve(10, "dg_01")
    assert e.reserve_or_wait(10, "dg_01") == "reserved"     # 내 것 재획득(멱등)


def test_reserve_or_wait_waits_when_held_no_cycle():
    e = _engine()
    e.try_reserve(10, "dg_01")                 # A 보유. A는 아무것도 안 기다림
    assert e.reserve_or_wait(10, "dg_02") == "waiting"      # B는 안전하게 대기


def test_reserve_or_wait_detects_deadlock():
    e = _engine()
    e.try_reserve(10, "dg_01")                 # A 보유 10
    e.try_reserve(11, "dg_02")                 # B 보유 11
    assert e.reserve_or_wait(10, "dg_02") == "waiting"      # B가 10 대기(등록)
    # 이제 A가 11(=B 보유)을 원함 → A→B→A 사이클 → 거절
    assert e.reserve_or_wait(11, "dg_01") == "deadlock"


def test_reserve_or_wait_reclaims_expired():
    clk = FakeClock()
    e = _engine(ttl=15.0, clock=clk)
    e.try_reserve(10, "dg_01")
    clk.advance(20.0)                          # TTL 초과 → 죽은 예약
    assert e.reserve_or_wait(10, "dg_02") == "reserved"
    assert e.holder_of(10) == "dg_02"


def test_reserve_or_wait_reserved_clears_prior_wait():
    e = _engine()
    e.try_reserve(10, "dg_01")
    assert e.reserve_or_wait(10, "dg_02") == "waiting"      # B 대기 등록
    e.release(10, "dg_01")                     # A가 놓음
    assert e.reserve_or_wait(10, "dg_02") == "reserved"     # B가 잡음
    # 잡은 뒤 B의 대기가 지워져 있어야 대기 그래프가 오염되지 않는다
    assert e.would_deadlock("dg_03", 10) is False          # 10=B보유, B는 이제 대기 안 함


# --------------------- 노드(자리) 예약 — RP-EX 노드 점유 --------------------- #
# 배경: 로봇이 차지하는 공간은 '통로 위' 아니면 '노드 위'다. 통로만 예약하면 노드에
# 서 있는 로봇이 교통관제에 안 보여, 남이 '통로는 비었으니 가도 된다'며 그 자리로
# 들어온다(정점 충돌). 노드를 '길이 0짜리 가상 통로'(id = -n)로 예약 대상에 넣어 막는다.

def test_node_slot_roundtrip():
    e = _engine()
    assert e.node_slot(7) == -7
    assert e.is_node_slot(-7) is True
    assert e.is_node_slot(11) is False
    assert e.node_of_slot(e.node_slot(7)) == 7


def test_node_slot_reserved_like_corridor():
    """자리도 통로와 똑같이 '한 번에 한 로봇'이다 — 예약 로직은 키의 의미를 모른다."""
    e = _engine()
    slot = e.node_slot(2)
    assert e.try_reserve(slot, "dg_01") is True
    assert e.try_reserve(slot, "dg_02") is False
    assert e.holder_of(slot) == "dg_01"
    assert e.release(slot, "dg_01") is True
    assert e.try_reserve(slot, "dg_02") is True


def test_node_slots_do_not_pollute_pathfinding():
    """가상 통로는 예약 전용 자원이지 '길'이 아니다 — _adj 에 들어가면 안 된다.

    들어가면 Dijkstra 가 '2에서 2로 가는 비용 0 간선'을 보고 경로에 중복 노드를 넣는다.
    """
    e = _engine()
    route = e.find_path(1, 3)
    assert route.nodes == (1, 2, 3) or route.nodes == (1, 4, 3)
    assert len(set(route.nodes)) == len(route.nodes)     # 같은 노드가 두 번 안 나온다
    assert all(cid > 0 for cid in route.corridors)       # 경로에 가상 통로가 없다


def test_zero_waypoint_id_rejected():
    """node_slot(0) == 0 이라 통로 0 과 키가 겹친다 → 조용히 덮어쓰느니 즉시 터뜨린다."""
    with pytest.raises(ValueError):
        RoutingEngine([0, 1], [{"corridor_id": 10, "a": 0, "b": 1}])


# ------------------------ blocked_nodes (지점 통째 회피) ------------------------ #

def test_blocked_nodes_detours_around_point():
    """지점 하나를 통째로 피한다. 통로만 막아서는 다른 통로로 같은 지점에 또 들어간다."""
    e = _engine()
    assert e.find_path(1, 3, blocked_nodes={2}).nodes == (1, 4, 3)
    assert e.find_path(1, 3, blocked_nodes={4}).nodes == (1, 2, 3)


def test_blocked_nodes_does_not_check_start():
    """내가 서 있는 자리가 막혀 있어도 거기서 출발은 해야 한다(이웃만 검사)."""
    e = _engine()
    assert e.find_path(1, 3, blocked_nodes={1, 2}).nodes == (1, 4, 3)


def test_blocked_nodes_goal_unreachable():
    e = _engine()
    assert e.find_path(1, 3, blocked_nodes={3}) is None


def test_blocked_nodes_all_routes_blocked():
    e = _engine()
    assert e.find_path(1, 3, blocked_nodes={2, 4}) is None


# ------------------ 자리를 포함한 데드락(대기 사이클) 검사 ------------------ #

def test_deadlock_cycle_through_node_slot():
    """자리를 자원으로 등록하면 '자리 대기'가 대기 사슬에 그대로 들어간다.

    would_deadlock 은 한 줄도 안 고쳤는데, 자원이 하나 늘어난 것만으로
    '서 있는 로봇 ↔ 지나가려는 로봇' 교착을 감지하게 된다.
    """
    e = _engine()
    slot2 = e.node_slot(2)
    e.try_reserve(slot2, "dg_01")          # dg_01 이 2번에 서 있다
    e.try_reserve(10, "dg_02")             # dg_02 가 통로10 을 쥐고
    e.begin_wait("dg_02", slot2)           # 2번 자리를 기다린다
    # dg_01 이 통로10 을 기다리면 사이클: dg_01 → 통로10(dg_02) → 자리2(dg_01)
    assert e.would_deadlock("dg_01", 10) is True


# ---------------------- reservation_snapshot (관측용 덤프) ---------------------- #

def test_reservation_snapshot_splits_corridors_and_nodes():
    """관측 쪽으로 음수 id 가 새면 화면이 통로 번호로 오해한다 → 엔진이 갈라서 준다."""
    e = _engine()
    e.try_reserve(10, "dg_01")
    e.try_reserve(e.node_slot(2), "dg_01")
    e.try_reserve(11, "dg_02")
    snap = e.reservation_snapshot()
    assert snap["corridors"] == {10: "dg_01", 11: "dg_02"}
    assert snap["nodes"] == {2: "dg_01"}                 # 키가 -2 가 아니라 2
    assert all(k > 0 for k in snap["nodes"])


def test_reservation_snapshot_hides_expired():
    """TTL 지난 죽은 예약은 관측에도 안 보인다(주기 회수가 아직 안 돈 사이에도)."""
    clock = FakeClock()
    e = _engine(ttl=15.0, clock=clock)
    e.try_reserve(10, "dg_01")
    e.try_reserve(e.node_slot(2), "dg_01")
    assert e.reservation_snapshot()["corridors"] == {10: "dg_01"}
    clock.advance(20.0)
    snap = e.reservation_snapshot()
    assert snap["corridors"] == {} and snap["nodes"] == {}


def test_reap_expired_collects_node_slots():
    """주기 회수가 자리도 걷어간다 — 유령 예약은 화면뿐 아니라 데드락 검사도 오염시킨다."""
    clock = FakeClock()
    e = _engine(ttl=15.0, clock=clock)
    e.try_reserve(11, "dg_02")
    e.try_reserve(e.node_slot(3), "dg_02")
    clock.advance(20.0)
    assert sorted(e.reap_expired()) == sorted([11, e.node_slot(3)])
    assert e.holder_of(11) is None
    assert e.holder_of(e.node_slot(3)) is None
