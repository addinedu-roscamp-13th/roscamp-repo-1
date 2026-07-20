#!/usr/bin/env python3
"""짝(pair) 지점 제자리 회전 촬영 단위테스트 — 로봇/DB 없이 하달 시퀀스만 검증.

배경:
  촬영 카메라가 로봇 한쪽에 고정돼 있어 통로를 한 번 지나면 한쪽 베드만 찍힌다.
  그래서 같은 자리에서 방향(yaw)만 180° 돌려 한 번 더 찍는다. 이 '같은 위치 다른 방향'
  지점을 waypoints.pair_waypoint_id 로 표현하고, corridors 에는 넣지 않는다
  (= 경로 탐색 대상이 아니다). 이동이 아니라 제자리 회전이기 때문이다.

여기서 검증하는 것:
  ① 부모 도착 직후에 짝이 '한 번 더' 하달되는가 (순서까지)
  ② 짝 Goal 의 x·y 가 부모와 완전히 같고 yaw 만 다른가
     (좌표가 다르면 로봇이 회전 대신 주행으로 분기해 좁은 통로에서 실패한다)
  ③ 짝 촬영이 실패하면 그 지점이 '방문 완료'로 집계되지 않는가
  ④ 로봇별 시작 노드(start_wp)가 전역 상수보다 우선하는가

테스트 그래프 (짝은 그래프에 없다):
    1 --100-- 2 --101-- 3 --102-- 4     2 의 짝 = 99 (좌표 동일, yaw 만 반대)

실행:
  PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pytest test/test_patrol_pair_capture.py -v
"""
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

    하달된 Goal 을 sent 에 순서대로 쌓아두고, 미리 정한 result_code 를 돌려준다.
    fail_on: 이 waypoint_id 가 마지막인 Goal 에는 실패(code=1)를 돌려준다.
    """
    def __init__(self, fail_on=None):
        self.sent = []          # [(waypoint_ids, capture_flags, goal), ...]
        self.fail_on = fail_on

    def wait_for_server(self, timeout_sec=None):
        return True

    def send_goal_async(self, goal, feedback_callback=None):
        wp_ids = [w.waypoint_id for w in goal.waypoints]
        caps = [w.capture for w in goal.waypoints]
        self.sent.append((wp_ids, caps, goal))
        last = wp_ids[-1]
        code = 1 if (self.fail_on is not None and last == self.fail_on) else 0
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
    status = d.run_patrol(1, "dg_01", waypoints, engine, client, start_wp=start_wp)
    return status, client.sent


# =========================================================================== #
# ① 부모 직후에 짝이 하달된다
# =========================================================================== #
def test_짝은_부모_도착_직후에_하달된다():
    status, sent = _run(FakeClient())
    hadal = [wp_ids for wp_ids, _caps, _g in sent]
    assert hadal == [[2], [99], [3]], f"하달 순서가 다르다: {hadal}"
    assert status == "COMPLETED"


def test_짝_Goal_은_촬영_플래그가_켜져_있다():
    _status, sent = _run(FakeClient())
    _wp_ids, caps, _g = sent[1]          # 두 번째 = 짝 하달
    assert caps == [True], "짝은 촬영이 목적이므로 capture=True 여야 한다"


def test_짝이_없는_지점은_추가_하달이_없다():
    """3 번은 pair_of 에 없으므로 도착 후 아무 것도 더 보내지 않는다.

    통로가 연속으로 예약되면 여러 노드가 한 Goal(세그먼트)로 묶여 나가는 것이 정상이므로,
    '몇 개의 Goal 이 갔나'가 아니라 '짝(99) 하달이 있었나'로 확인한다.
    """
    _status, sent = _run(FakeClient(), targets=(3,))
    hadal = [wp for wp, _c, _g in sent]
    assert not any(99 in wp for wp in hadal), f"짝이 없는데 추가 하달됨: {hadal}"
    assert hadal[-1][-1] == 3, f"목적지 3 에 도달하지 않았다: {hadal}"


# =========================================================================== #
# ② 짝 좌표는 부모와 완전히 같고 yaw 만 다르다
# =========================================================================== #
def test_짝_좌표는_부모와_동일하고_yaw_만_다르다():
    """좌표가 다르면 로봇이 제자리 회전(Spin)이 아니라 주행으로 분기해 실패한다."""
    _status, sent = _run(FakeClient())
    parent = sent[0][2].waypoints[-1]     # 2 번 도착 Goal 의 마지막 Waypoint
    pair = sent[1][2].waypoints[-1]       # 짝 99 Goal
    assert (pair.x, pair.y) == (parent.x, parent.y)
    assert pair.yaw != parent.yaw


# =========================================================================== #
# ③ 짝 촬영 실패 = 그 지점 미완
# =========================================================================== #
def test_짝_촬영이_실패하면_부분완료로_끝난다():
    """부모는 찍었지만 짝을 못 찍었으면 '다 돌았다'고 보고해선 안 된다.

    지점 3개(2·3·4) 중 2 만 짝 촬영에 실패한다 → 나머지 둘은 방문 성공이므로
    FAILED(사실상 아무 데도 못 감)가 아니라 COMPLETED_PARTIAL 이어야 한다.
    """
    client = FakeClient(fail_on=99)
    status, sent = _run(client, targets=(2, 3, 4))
    hadal = [wp for wp, _c, _g in sent]
    # 짝(99)은 최초 1회 + 마지막 재시도에서 1회, 총 2번 시도된다.
    assert hadal.count([99]) == 2, f"재시도가 없다: {hadal}"
    assert status == "COMPLETED_PARTIAL"


# =========================================================================== #
# ④ 로봇별 시작 노드가 전역 상수보다 우선한다
# =========================================================================== #
def test_start_wp_가_주어지면_그_노드에서_출발한다():
    """dg_01→22, dg_02→23 처럼 로봇마다 충전소가 다르다. 첫 Goal 이 출발점을 드러낸다."""
    client = FakeClient()
    _status, sent = _run(client, start_wp=1, targets=(3,))
    # 1 에서 출발했으므로 3 까지 2 를 경유한다(1-2-3). 예약이 이어지면 한 Goal 에 묶인다.
    first_goal_wps = sent[0][0]
    assert first_goal_wps[-1] == 3
    assert 2 in first_goal_wps, f"출발 노드 1 에서 경유해야 한다: {first_goal_wps}"
