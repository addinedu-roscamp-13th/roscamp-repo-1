#!/usr/bin/env python3
"""촬영 판정(문서 E2 20번)과 짝(pair) 제자리 회전 촬영 단위테스트.

로봇/DB 없이 '어떤 배열을 어떤 capture 플래그로 하달하는가'만 검증한다.

배경:
  촬영 카메라가 로봇 한쪽에 고정돼 있어 통로를 한 번 지나면 한쪽 베드만 찍힌다.
  그래서 같은 자리에서 방향(yaw)만 180° 돌려 한 번 더 찍는다. 이 '같은 위치 다른 방향'
  지점을 waypoints.pair_waypoint_id 로 표현하고, corridors 에는 넣지 않는다
  (= 경로 탐색 대상이 아니다). 이동이 아니라 제자리 회전이기 때문이다.

  하달은 문서 판정식을 노드마다 적용한다 — capture = (순찰 지점) AND (이번 task 미방문).
  짝이 있으면 같은 배열의 바로 뒤에 연달아 넣는다(별도 Goal 을 보내지 않는다).
  로봇은 직전 원소와 좌표가 같으면 주행이 아니라 제자리 회전으로 분기한다.

여기서 검증하는 것:
  ① 짝이 부모 바로 뒤에, 같은 배열로 하달되는가
  ② 짝 Goal 의 x·y 가 부모와 완전히 같고 yaw 만 다른가
     (좌표가 다르면 로봇이 회전 대신 주행으로 분기해 좁은 통로에서 실패한다)
  ③ 경로 중간에 지나가는 미방문 순찰 지점도 촬영하는가 (판정식의 핵심)
  ④ 이미 찍은 지점은 다시 찍지도, 다시 가지도 않는가
  ⑤ 짝을 못 찍으면 그 지점이 '방문 완료'로 집계되지 않는가
  ⑥ 로봇별 시작 노드(start_wp)가 전역 상수보다 우선하는가

테스트 그래프 (짝은 그래프에 없다):
    1 --100-- 2 --101-- 3 --102-- 4     2 의 짝 = 99 (좌표 동일, yaw 만 반대)

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

# 라우팅 그래프 — 짝(99)은 노드로도 통로로도 등장하지 않는다.
NODES = [1, 2, 3, 4]
CORRIDORS = [
    {"corridor_id": 100, "a": 1, "b": 2, "length": 1.0},
    {"corridor_id": 101, "a": 2, "b": 3, "length": 1.0},
    {"corridor_id": 102, "a": 3, "b": 4, "length": 1.0},
]
# 하달용 좌표표는 짝까지 전부 담는다(짝을 보내려면 좌표·yaw 가 필요하다).
# capture=True 가 '순찰 지점'을 뜻한다(1 번은 통로 경유점이라 False).
WP_META = {
    1: {"x": 0.0, "y": 0.0, "yaw": 0.0, "capture": False},
    2: {"x": 1.0, "y": 2.0, "yaw": -1.57, "capture": True},
    3: {"x": 3.0, "y": 0.0, "yaw": 0.0, "capture": True},
    4: {"x": 5.0, "y": 0.0, "yaw": 0.0, "capture": True},
    99: {"x": 1.0, "y": 2.0, "yaw": 1.57, "capture": True},   # 2 와 x·y 동일
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
        cb(self)            # 이미 끝났으므로 바로 호출

    def result(self):
        return self._value


class FakeClient:
    """Navigate 액션 클라이언트 대역.

    하달된 Goal 을 sent 에 순서대로 쌓아두고 결과를 돌려준다.
    stop_before: 배열에 이 waypoint_id 가 있으면 **그 앞에서 멈췄다**고 보고한다
                 (code=1, last_waypoint_id=직전 원소). 짝 촬영 실패 재현용.
    """
    def __init__(self, stop_before=None):
        self.sent = []          # [(waypoint_ids, capture_flags, goal), ...]
        self.stop_before = stop_before

    def wait_for_server(self, timeout_sec=None):
        return True

    def send_goal_async(self, goal, feedback_callback=None):
        wp_ids = [w.waypoint_id for w in goal.waypoints]
        caps = [w.capture for w in goal.waypoints]
        self.sent.append((wp_ids, caps, goal))
        if self.stop_before is not None and self.stop_before in wp_ids:
            i = wp_ids.index(self.stop_before)
            # 배열 첫 원소에서 막히면 아직 아무 데도 못 간 것이다.
            code, last = 1, (wp_ids[i - 1] if i > 0 else 0)
        else:
            code, last = 0, wp_ids[-1]
        result = SimpleNamespace(
            result=SimpleNamespace(result_code=code, last_waypoint_id=last))
        handle = SimpleNamespace(
            accepted=True, get_result_async=lambda: _Future(result))
        return _Future(handle)


def _make_dispatcher():
    d = PatrolDispatcher(_Logger())
    d.wp_meta = dict(WP_META)
    d.pair_of = {2: 99}          # 부모 2 의 짝은 99
    return d


def _run(client, start_wp=1, targets=(2, 3)):
    d = _make_dispatcher()
    engine = RoutingEngine(NODES, CORRIDORS)
    waypoints = [{"waypoint_id": t} for t in targets]
    status, _unvisited = d.run_patrol(
        1, "dg_01", waypoints, engine, client, start_wp=start_wp)
    return status, client.sent


def _captured(sent):
    """하달 기록에서 '촬영 플래그가 켜진 채 나간' waypoint_id 를 순서대로 모은다."""
    out = []
    for wp_ids, caps, _goal in sent:
        out.extend(w for w, c in zip(wp_ids, caps) if c)
    return out


# =========================================================================== #
# ① 짝은 부모 바로 뒤에, 같은 배열로 나간다
# =========================================================================== #
def test_짝은_부모_바로_뒤에_같은_배열로_하달된다():
    """별도 Goal 을 한 번 더 보내지 않는다(문서 E2 20-1)."""
    status, sent = _run(FakeClient())
    with_pair = [wp_ids for wp_ids, _c, _g in sent if 99 in wp_ids]
    assert with_pair, f"짝이 하달되지 않았다: {[w for w, _c, _g in sent]}"
    for wp_ids in with_pair:
        i = wp_ids.index(99)
        assert i > 0 and wp_ids[i - 1] == 2, \
            f"짝 99 가 부모 2 바로 뒤가 아니다: {wp_ids}"
    assert not any(wp_ids == [99] for wp_ids, _c, _g in sent), \
        "짝만 담은 별도 Goal 이 나갔다(옛 방식)"
    assert status == "COMPLETED"


def test_짝_Goal_은_촬영_플래그가_켜져_있다():
    _status, sent = _run(FakeClient())
    for wp_ids, caps, _g in sent:
        if 99 in wp_ids:
            assert caps[wp_ids.index(99)] is True, \
                "짝은 촬영이 목적이므로 capture=True 여야 한다"


# =========================================================================== #
# ② 짝 좌표는 부모와 완전히 같고 yaw 만 다르다
# =========================================================================== #
def test_짝_좌표는_부모와_동일하고_yaw_만_다르다():
    """좌표가 다르면 로봇이 제자리 회전(Spin)이 아니라 주행으로 분기해 실패한다."""
    _status, sent = _run(FakeClient())
    goal = next(g for ids, _c, g in sent if 99 in ids)
    ids = [w.waypoint_id for w in goal.waypoints]
    parent = goal.waypoints[ids.index(2)]
    pair = goal.waypoints[ids.index(99)]
    assert (pair.x, pair.y) == (parent.x, parent.y)
    assert pair.yaw != parent.yaw


# =========================================================================== #
# ③ 경로 중간의 미방문 순찰 지점도 찍는다 (판정식의 핵심)
# =========================================================================== #
def test_지나가는_길의_미방문_순찰지점도_촬영한다():
    """capture = 순찰지점 AND 미방문 — '배열의 마지막 하나만' 이 아니다.

    1 에서 4 로 가려면 2·3 을 지난다. 둘 다 아직 안 찍은 순찰 지점이므로
    지나는 김에 찍어야 나중에 그 지점을 목표로 다시 오지 않는다.
    """
    _status, sent = _run(FakeClient(), targets=(4,))
    captured = _captured(sent)
    assert 2 in captured and 3 in captured, \
        f"지나가는 순찰 지점을 안 찍었다: {[(w, c) for w, c, _g in sent]}"
    assert 99 in captured, "중간에 찍은 지점의 짝도 같이 찍어야 한다"
    assert 1 not in captured, "순찰 지점이 아닌 통로 경유점을 찍었다"


# =========================================================================== #
# ④ 이미 찍은 지점은 다시 찍지도, 다시 가지도 않는다
# =========================================================================== #
def test_이미_찍은_지점은_다시_찍지_않는다():
    """4 로 가는 길에 2 를 찍었으면, 그 뒤 목표 2 는 이동조차 하지 않는다."""
    status, sent = _run(FakeClient(), targets=(4, 2))
    captured = _captured(sent)
    assert captured.count(2) == 1, f"같은 지점을 두 번 찍었다: {captured}"
    assert captured.count(99) == 1, f"짝을 두 번 찍었다: {captured}"
    assert status == "COMPLETED"


# =========================================================================== #
# ⑤ 짝을 못 찍으면 그 지점은 미완이다
# =========================================================================== #
def test_짝_촬영이_실패하면_부분완료로_끝난다():
    """부모는 찍었지만 짝을 못 찍었으면 '다 돌았다'고 보고해선 안 된다.

    지점 3개(2·3·4) 중 2 의 짝만 계속 실패한다 → 나머지는 방문 성공이므로
    COMPLETED 가 아니라 COMPLETED_PARTIAL 이어야 한다.
    """
    client = FakeClient(stop_before=99)
    status, sent = _run(client, targets=(2, 3, 4))
    tried = [ids for ids, _c, _g in sent if 99 in ids]
    assert len(tried) >= 2, f"짝 촬영 재시도가 없다: {[i for i, _c, _g in sent]}"
    assert status == "COMPLETED_PARTIAL"


# =========================================================================== #
# ⑥ 로봇별 시작 노드가 전역 상수보다 우선한다
# =========================================================================== #
def test_start_wp_가_주어지면_그_노드에서_출발한다():
    """start_wp=1 이면 1 에서 출발해 2 로 이동한다(첫 구간부터 통로 예약 보호)."""
    _status, sent = _run(FakeClient(), start_wp=1, targets=(2,))
    first_ids = sent[0][0]
    assert first_ids[0] == 2, f"첫 하달이 출발 노드의 다음이 아니다: {first_ids}"


# =========================================================================== #
# ⑦ 통과·미촬영 노드는 '진행 방향'을 향한다 (두리번거림 제거, RP-EX)
# =========================================================================== #
# 배경: 예전에는 촬영 지점이 아닌 노드의 yaw 를 무조건 0(동쪽)으로 강제했다.
# 그래서 로봇이 그냥 지나가면 될 노드에서도 동쪽으로 고개를 돌려, 실측에서 이동
# 1m 당 1500° 넘게 회전(두리번거림)했다. 촬영은 capture=true 노드에서만 하므로,
# 그 외 노드는 '가는 방향(다음 노드 쪽)'을 향하게 해 불필요한 회전을 없앤다.
def test__travel_yaw_는_다음_노드_쪽을_향한다():
    coords = [(0.0, 0.0), (1.0, 1.0), (2.0, 0.0)]
    assert PatrolDispatcher._travel_yaw(coords, 0) == math.atan2(1.0, 1.0)   # 다음(1,1)
    assert PatrolDispatcher._travel_yaw(coords, 1) == math.atan2(-1.0, 1.0)  # 다음(2,0)
    # 마지막 노드는 다음이 없으므로 '오던 방향'을 유지한다.
    assert PatrolDispatcher._travel_yaw(coords, 2) == math.atan2(-1.0, 1.0)


def test__travel_yaw_같은자리거나_한점이면_0():
    """두 점이 같으면(짝 등) 진행 방향 계산이 불가 → 0.0 폴백."""
    assert PatrolDispatcher._travel_yaw([(1.0, 2.0), (1.0, 2.0)], 0) == 0.0
    assert PatrolDispatcher._travel_yaw([(5.0, 5.0)], 0) == 0.0


def test_통과노드는_0이_아니라_진행방향으로_하달된다():
    """통과 노드(1)를 지나 촬영 노드(2)로 가는 배열을 직접 하달해 yaw 를 확인한다."""
    d = _make_dispatcher()
    client = FakeClient()
    # 1=(0,0) 통과점, 2=(1,2) 촬영점. capture_ids 에 2 만 넣는다.
    d._dispatch_segment(client, 1, [1, 2], {2})
    _ids, _caps, goal = client.sent[0]
    passthru, capture = goal.waypoints[0], goal.waypoints[1]
    assert passthru.capture is False and capture.capture is True
    # 통과점: 다음 노드(1,2) 쪽 = atan2(2,1). 예전의 0(동쪽)이 아니다.
    assert passthru.yaw == math.atan2(2.0, 1.0)
    assert passthru.yaw != 0.0
    # 촬영점: DB 의 베드 방향(-1.57)을 그대로 유지한다.
    assert capture.yaw == -1.57
