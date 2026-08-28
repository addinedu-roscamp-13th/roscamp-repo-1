#!/usr/bin/env python3
"""촬영 판정(방향 게이트)과 짝(pair) 처리 단위테스트 (RP-EX).

로봇/DB 없이 '어떤 배열을 어떤 capture 플래그로 하달하는가'만 검증한다.

배경 — 옛 방식에서 무엇이 바뀌었나:
  촬영 카메라가 로봇 옆 한쪽에 고정돼 있어, 통로를 지나는 방향이 곧 어느 베드를
  찍느냐다. 예전에는 '부모 지점 도착 → 제자리 180° 회전 → 반대쪽 촬영(짝)'으로 처리
  했으나, 그 제자리 180° 회전이 좁은 통로에서 물리적으로 불가능함이 현장에서 확인됐다.
  이제 같은 자리의 반대 방향 촬영(짝)은 **반대 방향으로 지날 때** 찍는다(왕복 촬영).

  그래서 촬영은 '지나는 방향에 맞을 때만' 한다(방향 게이트):
    · 노드 자신의 촬영 방향이 진행 방향과 맞으면 그 노드를 찍는다.
    · 아니면 그 자리의 짝(반대 방향)이 진행 방향과 맞으면 짝을 찍는다(하달 배열에 짝 id).
    · 둘 다 아니면 통과.
  짝은 corridors 에 없어(경로 탐색 대상 아님) 부모 노드로 경로를 찾고, 좌표는 짝 것을
  쓴다(부모와 6~8cm 떨어진 '렌즈 보정' 위치).

여기서 검증하는 것:
  ① 남향으로 지나면 남향 지점(부모)을, 북향으로 지나면 짝을 찍는다(방향 게이트)
  ② 180° 뒤돌아 찍어야 하는 지점은 그 방향에선 안 찍는다(제자리 회전 금지)
  ③ 짝 하달 배열에는 '짝 id'가 들어가고 좌표·yaw 는 짝 것이다
  ④ 짝은 부모 노드로 경로를 찾는다(_parent_of), 목표로 승격돼도 탐색이 안 깨진다
  ⑤ 이미 찍은 지점은 다시 찍지 않는다
  ⑥ entry_wp 를 주면 언도킹+진입 노드 경유가, 안 주면 곧장 순찰이 일어난다

테스트 그래프 (짝 99 는 그래프에 없다):
    1 --100-- 2 --101-- 3    2 의 짝 = 99 (2 와 같은 자리, 반대 방향 촬영)
  좌표는 세로 통로를 흉내내 y축에 둔다 → 남/북 주행으로 방향 게이트를 시험한다.

실행:
  PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pytest test/test_patrol_pair_capture.py -v
"""
import math
import os
import sys
from types import SimpleNamespace

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from automato_control_service.patrol_dispatcher import PatrolDispatcher  # noqa: E402
from automato_control_service.routing_engine import RoutingEngine  # noqa: E402

SOUTH = -math.pi / 2      # -y 방향으로 지날 때의 촬영 방향(남향)
NORTH = math.pi / 2       # +y 방향(북향)

# 세로 통로: 1(맨 아래) — 2(가운데, 순찰점) — 3(맨 위). y 로만 늘어놓는다.
NODES = [1, 2, 3]
CORRIDORS = [
    {"corridor_id": 100, "a": 1, "b": 2, "length": 1.0},
    {"corridor_id": 101, "a": 2, "b": 3, "length": 1.0},
]
# 2 = 남향 촬영 순찰점. 99 = 2 의 짝(북향 촬영), 2 보다 살짝 위(렌즈 보정 7cm).
# 1·3 은 통로 경유점(capture=False).
WP_META = {
    1: {"x": 0.0, "y": 0.0, "yaw": 0.0, "capture": False},
    2: {"x": 0.0, "y": 1.0, "yaw": SOUTH, "capture": True},
    3: {"x": 0.0, "y": 2.0, "yaw": 0.0, "capture": False},
    99: {"x": 0.0, "y": 1.07, "yaw": NORTH, "capture": True},   # 2 의 짝(북향)
}


class _Logger:
    """ROS 로거 대역 — 호출만 받아 삼킨다."""
    def info(self, *_a, **_k): pass
    def warn(self, *_a, **_k): pass
    def error(self, *_a, **_k): pass
    def debug(self, *_a, **_k): pass


class _Future:
    """rclpy Future 대역 — 이미 완료된 상태로 굴어 테스트가 즉시 진행되게 한다."""
    def __init__(self, value):
        self._value = value

    def add_done_callback(self, cb):
        cb(self)

    def result(self):
        return self._value


class FakeClient:
    """Navigate 액션 클라이언트 대역. 하달된 Goal 을 sent 에 쌓고 도착으로 처리한다."""
    def __init__(self):
        self.sent = []          # [(waypoint_ids, capture_flags, goal), ...]

    def wait_for_server(self, timeout_sec=None):
        return True

    def send_goal_async(self, goal, feedback_callback=None):
        wp_ids = [w.waypoint_id for w in goal.waypoints]
        caps = [w.capture for w in goal.waypoints]
        self.sent.append((wp_ids, caps, goal))
        result = SimpleNamespace(
            result=SimpleNamespace(result_code=0, last_waypoint_id=wp_ids[-1]))
        handle = SimpleNamespace(
            accepted=True, get_result_async=lambda: _Future(result))
        return _Future(handle)


def _make_dispatcher():
    d = PatrolDispatcher(_Logger())
    d.wp_meta = dict(WP_META)
    d.pair_of = {2: 99}          # 부모 2 의 짝은 99
    return d


def _run(client, start_wp=1, entry_wp=None, targets=(2,)):
    d = _make_dispatcher()
    engine = RoutingEngine(NODES, CORRIDORS)
    waypoints = [{"waypoint_id": t} for t in targets]
    status, unvisited, _last = d.run_patrol(
        1, "dg_01", waypoints, engine, client,
        start_wp=start_wp, entry_wp=entry_wp)
    return status, unvisited, client.sent


def _captured(sent):
    """하달 기록에서 '촬영 플래그가 켜진 채 나간' waypoint_id 를 순서대로 모은다."""
    out = []
    for wp_ids, caps, _goal in sent:
        out.extend(w for w, c in zip(wp_ids, caps) if c)
    return out


# =========================================================================== #
# 방향 게이트 단위 — 진행 방향에 맞는 것만 고른다
# =========================================================================== #
def test_남향으로_지나면_부모를_찍는다():
    """1(아래)→2 로 올라가는 게 아니라, 3(위)→2 로 내려가면(남향) 부모 2 를 찍는다."""
    d = _make_dispatcher()
    # seg_start=3(위), seg_wps=[2] → 진행 방향 남향(-y). 2 의 촬영 방향도 남향 → 부모.
    hadal, cap_ids, parents = d._build_segment_goal([2], set(), seg_start=3)
    assert hadal == [2] and cap_ids == {2} and parents == [2]


def test_북향으로_지나면_짝을_찍는다():
    """1(아래)→2 로 올라가면(북향) 부모(남향)는 못 찍고, 짝 99(북향)를 찍는다."""
    d = _make_dispatcher()
    hadal, cap_ids, parents = d._build_segment_goal([2], set(), seg_start=1)
    assert cap_ids == {99}, f"북향 진행이면 짝을 찍어야 한다: {cap_ids}"
    assert hadal == [99], "하달 배열에는 짝 id 가 들어간다(좌표가 짝 것)"
    assert parents == [99]


def test_짝은_180도_뒤돌아_찍지_않는다():
    """부모(남향)를 북향으로 지날 때, 부모는 후보에서 빠진다(제자리 180° 회전 금지)."""
    d = _make_dispatcher()
    # 짝을 이미 찍었다고 두면, 북향 진행에서 부모는 방향이 안 맞아 아무것도 안 찍는다.
    hadal, cap_ids, _parents = d._build_segment_goal([2], {99}, seg_start=1)
    assert cap_ids == set(), f"180° 뒤돌아 부모를 찍으면 안 된다: {cap_ids}"
    assert hadal == [2], "그래도 노드는 통과한다(capture=false)"


def test_방향_불명이면_안_찍는다():
    """seg_start 가 없으면(진입 방향 불명) 엉뚱한 방향으로 찍느니 통과시킨다."""
    d = _make_dispatcher()
    hadal, cap_ids, _parents = d._build_segment_goal([2], set(), seg_start=None)
    assert cap_ids == set() and hadal == [2]


# =========================================================================== #
# ③ 짝 하달 배열의 좌표·yaw 는 짝 것이다
# =========================================================================== #
def test_짝_하달은_짝의_좌표와_yaw_를_쓴다():
    """북향으로 2 를 지나며 짝 99 를 찍을 때, Goal 좌표·yaw 가 99 것이어야 한다."""
    client = FakeClient()
    # 1(아래)에서 출발해 3(위)로 올라가면 2 를 북향으로 지난다 → 짝 99 촬영.
    _status, _unv, sent = _run(client, start_wp=1, targets=(3,))
    goal = next(g for ids, _c, g in sent if 99 in ids)
    pair = goal.waypoints[[w.waypoint_id for w in goal.waypoints].index(99)]
    assert pair.capture is True
    assert (pair.x, pair.y) == (WP_META[99]["x"], WP_META[99]["y"])
    assert pair.yaw == NORTH


# =========================================================================== #
# ④ 짝을 목표로 줘도 경로 탐색이 부모로 돌아 깨지지 않는다
# =========================================================================== #
def test_짝을_목표로_줘도_부모로_경로를_찾는다():
    """짝(99)이 목표여도 find_path(99)로 실패하지 않고 부모(2)로 이동해 찍는다."""
    client = FakeClient()
    # 1→...→99(목표). _parent_of(99)=2 로 경로를 찾아 2 를 북향으로 지나며 짝을 찍는다.
    status, _unv, sent = _run(client, start_wp=1, targets=(99,))
    assert status == "COMPLETED"
    assert 99 in _captured(sent), "짝 목표가 촬영되지 않았다"


def test_parent_of_는_짝을_부모로_되돌린다():
    d = _make_dispatcher()
    assert d._parent_of(99) == 2      # 짝 → 부모
    assert d._parent_of(2) == 2       # 부모는 그대로
    assert d._parent_of(3) == 3       # 짝 아님 → 그대로


# =========================================================================== #
# ⑤ 이미 찍은 지점은 다시 찍지 않는다
# =========================================================================== #
def test_이미_찍은_지점은_다시_찍지_않는다():
    """visited 에 든 지점은 방향이 맞아도 후보에서 뺀다."""
    d = _make_dispatcher()
    hadal, cap_ids, _p = d._build_segment_goal([2], {2, 99}, seg_start=3)
    assert cap_ids == set() and hadal == [2]


# =========================================================================== #
# ⑥ entry_wp: 언도킹 + 진입 노드 경유
# =========================================================================== #
def test_entry_wp_없으면_곧장_순찰한다():
    """entry_wp 를 안 주면 언도킹·진입 노드 경유 없이 첫 목표로 바로 간다."""
    client = FakeClient()
    _status, _unv, sent = _run(client, start_wp=1, entry_wp=None, targets=(3,))
    # 첫 하달이 출발 노드(1)의 다음(2 또는 짝)으로 시작한다 — [1] 단독(언도킹)이 없다.
    assert sent[0][0][0] != 1 or len(sent[0][0]) > 1, \
        f"언도킹 하달이 잘못 나갔다: {sent[0][0]}"


def test_entry_wp_주면_언도킹_하달이_먼저_나간다():
    """entry_wp 를 주면 시작 노드 한 개짜리 언도킹 하달이 맨 앞에 나간다."""
    client = FakeClient()
    # start_wp=1, entry_wp=1 → 언도킹으로 [1] 하달(진입 노드가 시작과 같아 경유는 생략).
    _status, _unv, sent = _run(client, start_wp=1, entry_wp=1, targets=(3,))
    assert sent[0][0] == [1], f"맨 앞이 언도킹 [1] 이 아니다: {sent[0][0]}"
    assert sent[0][1] == [False], "언도킹은 촬영하지 않는다"
