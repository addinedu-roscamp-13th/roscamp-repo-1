#!/usr/bin/env python3
"""RP-EX 노드 점유 — '지점에 서 있는 로봇을 남이 관통하는' 결함의 회귀 테스트.

발단(실제로 겪은 상황):
  dg_01 이 7번에서 통로(7-8)를 기다리는 중인데, 다른 로봇이 10 → 7 → 8 로 지나가려 했다.
  7번엔 이미 dg_01 이 서 있는데도 교통관제가 아무 위반도 못 봤다 — 예약 자원이 통로뿐이라
  '자리'가 예약표에 아예 없었기 때문이다.

여기서 지키는 불변식:
  1. 남이 서 있는 지점을 지나는 세그먼트는 확보되지 않는다(관통 금지).
  2. 확보에 실패하면 이미 잡은 통로를 되뱉는다(가지도 못하면서 길만 막지 않는다).
  3. 실패한 지점은 '통째로' 회피해 재계획된다(통로 하나만 빼면 다른 통로로 또 들어간다).

테스트 그래프 — 실제 온실 맵에서 이 결함이 난 부분만 떼어 왔다(노드 번호도 실물과 같다):

      3 ---c5--- 8 ---c12--- 11
                 |            |
                c10          c15
                 |            |
   6 ---c9--- 7 --+           |
                 |            |
                c11          c18
                 |            |
                10 ---c14--- 13 ---c17--- 16

  10 → 8 최단경로는 7 경유(c11, c10)이고, 7이 막히면 13-16-…-11 로 크게 돈다.

실행:
  PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pytest test/test_node_occupancy.py -v
"""
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from automato_control_service import patrol_dispatcher as pd            # noqa: E402
from automato_control_service.patrol_dispatcher import PatrolDispatcher  # noqa: E402
from automato_control_service.routing_engine import RoutingEngine        # noqa: E402

WAYPOINTS = [3, 6, 7, 8, 10, 11, 13, 16]
CORRIDORS = [
    {"corridor_id": 5,  "a": 3,  "b": 8},
    {"corridor_id": 9,  "a": 6,  "b": 7},
    {"corridor_id": 10, "a": 7,  "b": 8},
    {"corridor_id": 11, "a": 7,  "b": 10},
    {"corridor_id": 12, "a": 8,  "b": 11},
    {"corridor_id": 14, "a": 10, "b": 13},
    {"corridor_id": 15, "a": 11, "b": 16},
    {"corridor_id": 17, "a": 13, "b": 16},
]


class _Log:
    """로그를 모아두는 가짜 로거. 어떤 판단을 했는지 문장으로 확인할 때 쓴다."""
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


@pytest.fixture
def fast_reserve(monkeypatch):
    """예약 대기를 짧게 줄인다(기본 30초를 그대로 기다릴 수는 없다).

    모듈 상수를 직접 바꾸는 이유: patrol_dispatcher 가 from-import 로 값을 이미
    바인딩해 두었기 때문에 patrol_config 를 고쳐도 반영되지 않는다.
    """
    monkeypatch.setattr(pd, "RESERVE_WAIT_SEC", 0.15)
    monkeypatch.setattr(pd, "RESERVE_POLL_SEC", 0.02)


@pytest.fixture
def env():
    engine = RoutingEngine(WAYPOINTS, CORRIDORS, reservation_ttl=60.0)
    log = _Log()
    return engine, PatrolDispatcher(log), log


def _occupy(engine, node, robot):
    """robot 이 node 에 서 있는 상태를 만든다(그 자리를 쥔다)."""
    assert engine.try_reserve(engine.node_slot(node), robot) is True


# --------------------------------------------------------------------------- #
def test_segment_not_acquired_through_occupied_node(env, fast_reserve):
    """불변식 1 — 남이 서 있는 7번을 지나는 세그먼트는 확보되지 않는다(관통 금지)."""
    engine, disp, _log = env
    _occupy(engine, 7, "dg_01")

    route = disp._plan_route(engine, 10, 8, set())
    assert route.nodes == (10, 7, 8)                  # 최단경로는 7 경유가 맞고

    seg = disp._acquire_segment(engine, "dg_02", route.hops(), set())
    assert seg is None, "7번에 남이 서 있는데 세그먼트를 확보했다(관통)"


def test_corridor_released_when_slot_unavailable(env, fast_reserve):
    """불변식 2 — 통로만 잡고 자리를 못 얻으면 그 통로를 도로 뱉는다.

    안 뱉으면 '가지도 못하면서 길만 막는' 로봇이 되어 상대까지 묶인다.
    """
    engine, disp, _log = env
    _occupy(engine, 7, "dg_01")

    disp._acquire_segment(engine, "dg_02", [(7, 11), (8, 10)], set())
    assert engine.holder_of(11) is None, "진입 통로를 쥔 채 놓지 않았다"


def test_occupied_node_is_avoided_wholesale(env, fast_reserve):
    """불변식 3 — 실패한 지점은 통째로 회피해 재계획된다.

    통로 하나만 블랙리스트에 넣는 방식이면 우회로가 다른 통로로 같은 지점에 또 들어간다.
    """
    engine, disp, _log = env
    _occupy(engine, 7, "dg_01")

    attempt_block = set()
    route = disp._plan_route(engine, 10, 8, attempt_block)
    assert disp._acquire_segment(engine, "dg_02", route.hops(), attempt_block) is None

    detour = disp._plan_route(engine, 10, 8, attempt_block)
    assert detour is not None, "우회로가 있는데 경로를 못 찾았다"
    assert 7 not in detour.nodes, f"우회로가 또 7을 지난다: {detour.nodes}"


def test_holder_keeps_its_slot(env, fast_reserve):
    """서 있던 로봇의 자리는 상대의 시도에 흔들리지 않는다."""
    engine, disp, _log = env
    _occupy(engine, 7, "dg_01")

    route = disp._plan_route(engine, 10, 8, set())
    disp._acquire_segment(engine, "dg_02", route.hops(), set())
    assert engine.holder_of(engine.node_slot(7)) == "dg_01"


def test_free_node_is_acquired_with_its_slot(env, fast_reserve):
    """아무도 없으면 통로와 도착 자리를 '쌍으로' 함께 확보한다."""
    engine, disp, _log = env

    route = disp._plan_route(engine, 10, 8, set())
    seg = disp._acquire_segment(engine, "dg_02", route.hops(), set())

    assert seg is not None
    seg_wps, seg_cids = seg
    assert seg_wps == [7, 8] and seg_cids == [11, 10]
    for wp in seg_wps:                                # 지나갈 자리도 전부 내 것
        assert engine.holder_of(engine.node_slot(wp)) == "dg_02"


def test_log_names_the_point_not_the_negative_id(env, fast_reserve):
    """로그가 '통로 -7' 이 아니라 '지점 7 자리' 로 읽혀야 사람이 원인을 안다."""
    engine, disp, log = env
    _occupy(engine, 7, "dg_01")

    route = disp._plan_route(engine, 10, 8, set())
    disp._acquire_segment(engine, "dg_02", route.hops(), set())

    assert log.has("지점 7 자리"), f"로그에 지점이 안 드러난다: {log.lines}"
    assert not log.has("통로 -7"), "음수 자원 id 가 로그로 샜다"


def test_blacklist_view_splits_corridors_and_points(env):
    """블랙리스트에 섞인 통로/자리를 화면 쪽으로 갈라서 내보낸다."""
    engine, disp, _log = env
    disp._blacklist_add(11)
    disp._blacklist_add(engine.node_slot(7))

    view = disp.blacklist_view(engine)
    assert view["corridors"] == [11]
    assert view["nodes"] == [7]
