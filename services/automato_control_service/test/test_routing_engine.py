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
