#!/usr/bin/env python3
"""RP-EX 교통관제 회피 목록 — '남의 사정과 통신 사고가 지도를 지우는' 결함의 회귀 테스트.

발단(2026-08-03 실사고 + 다중 로봇 재현):
  ① 중계 노드(DCS)가 둘이 되어 하달 5초 만에 가짜 실패가 돌아왔다. ACS 는 그것을
     '통로가 막혔다'로 믿고 블랙리스트에 넣었고, 세그먼트마다 반복되며 멀쩡한 통로가
     지도에서 하나씩 지워졌다 → '경로 없음' 연쇄 → 순찰이 110초 만에 조기 종료
     (촬영 12곳 중 6곳만 하고 복귀).
  ② dg_02 가 지점 9 에 서 있어 dg_03 이 양보했을 뿐인데, 그 회피가 전역이라
     아무 상관 없는 dg_01 의 9 행 3건이 전부 차단됐다.

여기서 지키는 불변식:
  1. 하달하자마자 온 실패는 막힘으로 치지 않는다(로봇이 시도해 볼 시간조차 없었다).
  2. 그 관용은 통로당 1회뿐이다 — 두 번째도 빠르면 정상 처리한다(무한 재시도 방지).
  3. 시간이 충분히 지난 실패는 예전처럼 곧바로 막힘으로 처리한다.
  4. 양보(남이 쓰는 중)는 그 로봇에게만 보이고, 진짜 막힘은 모두에게 보인다.
  5. 목표 자리가 회피 목록에 있다는 이유로 목표 자체를 포기하지 않는다.
  6. 단 attempt_block(이번 시도에서 방금 못 간 자리)은 그대로 차단한다(무한 왕복 방지).

테스트 그래프 — test_node_occupancy.py 와 같은 부분 맵(노드 번호도 실물과 같다):

      3 ---c5--- 8 ---c12--- 11
                 |            |
                c10          c15
                 |            |
   6 ---c9--- 7 --+           |
                 |            |
                c11          c18
                 |            |
                10 ---c14--- 13 ---c17--- 16

  10 → 8 최단경로는 7 경유(c11, c10)이고, 7이 막히면 13-16-11 로 크게 돈다.

실행:
  PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pytest test/test_traffic_blacklist.py -v
"""
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from automato_control_service import route_runner as rr                 # noqa: E402
from automato_control_service.route_runner import RouteRunner            # noqa: E402
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
    """예약 대기를 짧게 줄인다(기본 10초를 그대로 기다릴 수는 없다).

    모듈 상수를 직접 바꾸는 이유: route_runner 가 from-import 로 값을 이미
    바인딩해 두었기 때문에 patrol_config 를 고쳐도 반영되지 않는다.
    """
    monkeypatch.setattr(rr, "RESERVE_WAIT_SEC", 0.15)
    monkeypatch.setattr(rr, "RESERVE_POLL_SEC", 0.02)


@pytest.fixture
def env():
    engine = RoutingEngine(WAYPOINTS, CORRIDORS, reservation_ttl=60.0)
    log = _Log()
    return engine, RouteRunner(log), log


def _drive_returning(runner, monkeypatch, codes):
    """_dispatch_segment 를 '정해진 결과를 차례로 뱉는' 가짜로 바꾼다.

    실제 하달(액션 클라이언트·하트비트·룩어헤드)은 전부 이 함수 안에서 일어나므로,
    이것만 갈아끼우면 로봇도 ROS 도 없이 drive() 의 판정 흐름만 시험할 수 있다.
    codes 를 다 쓰면 2(취소)를 돌려 drive 를 확실히 끝낸다 — 안 그러면 우회로를
    찾아 계속 도느라 테스트가 안 끝난다.
    반환: 호출 기록 리스트(몇 번 하달했는지 확인용).
    """
    calls = []

    def fake_dispatch(client, task_id, waypoint_ids, capture_ids, **kwargs):
        calls.append(list(waypoint_ids))
        idx = len(calls) - 1
        return (codes[idx] if idx < len(codes) else 2), None

    monkeypatch.setattr(runner, "_dispatch_segment", fake_dispatch)
    return calls


# ---------------------------- 불변식 1~3: 가짜 실패 방어 ---------------------------- #
def test_quick_failure_does_not_blacklist(env, fast_reserve, monkeypatch):
    """불변식 1 — 하달하자마자 온 실패로는 통로를 지도에서 지우지 않는다.

    이걸 어기면 통신 사고 한 번에 멀쩡한 통로가 사라지고 '경로 없음'이 연쇄한다.
    """
    engine, runner, log = env
    # 무엇이든 '너무 빨리 온 실패'로 보이게 한다(경과 시간이 늘 이 값보다 작다).
    monkeypatch.setattr(rr, "MIN_BLOCK_ELAPSED_SEC", 10_000.0)
    _drive_returning(runner, monkeypatch, [1])      # 1회 실패 후 곧바로 중단

    runner.drive(engine, None, 1, "dg_01", 10, 8)

    assert log.has("세그먼트 실패가 너무 빠름"), f"관용 판정이 안 보인다: {log.lines}"
    assert not log.has("블랙리스트 후 우회"), "빠른 실패인데 블랙리스트에 넣었다"
    assert runner._blacklist_active("dg_01") == set(), "회피 목록이 오염됐다"


def test_second_quick_failure_is_blacklisted(env, fast_reserve, monkeypatch):
    """불변식 2 — 봐주는 것은 통로당 한 번뿐. 두 번째도 빠르면 정상 처리한다.

    무한히 봐주면 진짜 문제일 때 같은 통로로 영원히 재시도한다.
    """
    engine, runner, log = env
    monkeypatch.setattr(rr, "MIN_BLOCK_ELAPSED_SEC", 10_000.0)
    calls = _drive_returning(runner, monkeypatch, [1, 1])   # 같은 통로에서 두 번 실패

    runner.drive(engine, None, 1, "dg_01", 10, 8)

    assert len(calls) >= 2, "첫 실패 뒤 재시도를 안 했다(관용이 동작하지 않음)"
    assert calls[0] == calls[1], f"재계획이 다른 경로로 갔다: {calls[:2]}"
    assert log.has("블랙리스트 후 우회"), f"두 번째도 봐줬다: {log.lines}"


def test_slow_failure_is_blacklisted_immediately(env, fast_reserve, monkeypatch):
    """불변식 3 — 충분히 시간이 지난 실패는 예전처럼 곧바로 막힘으로 본다.

    관용 장치가 '진짜 막힘'까지 무디게 만들면 안 된다.
    """
    engine, runner, log = env
    # 0 으로 두면 어떤 실패든 '시간이 충분히 지난' 것으로 판정된다.
    monkeypatch.setattr(rr, "MIN_BLOCK_ELAPSED_SEC", 0.0)
    _drive_returning(runner, monkeypatch, [1])

    runner.drive(engine, None, 1, "dg_01", 10, 8)

    assert log.has("블랙리스트 후 우회"), f"진짜 막힘을 그냥 넘겼다: {log.lines}"
    assert not log.has("세그먼트 실패가 너무 빠름")


# ---------------------------- 불변식 4: 회피 목록의 주인 ---------------------------- #
def test_yield_is_private_to_the_robot(env):
    """불변식 4 — 양보(남이 쓰는 중)는 그 로봇에게만 보인다."""
    engine, runner, _log = env
    runner._blacklist_add(11, "dg_03")              # dg_03 이 통로 11 을 양보했다

    assert 11 in runner._blacklist_active("dg_03"), "정작 본인이 못 본다"
    assert 11 not in runner._blacklist_active("dg_01"), "남의 사정이 나를 막는다"
    assert 11 not in runner._blacklist_active(), "양보가 전역으로 샜다"


def test_real_block_is_shared_by_everyone(env):
    """불변식 4 — 진짜 막힘(주인 없음)은 모두가 피해야 한다.

    물리적 장애물은 누가 먼저 부딪혔든 모두에게 해당하기 때문이다.
    """
    engine, runner, _log = env
    runner._blacklist_add(11)                       # 주인 없음 = 주행이 실패한 진짜 막힘

    assert 11 in runner._blacklist_active("dg_01")
    assert 11 in runner._blacklist_active("dg_03")
    assert 11 in runner._blacklist_active(), "전역 막힘이 전역으로 안 보인다"


def test_others_yield_does_not_steal_my_route(env):
    """불변식 4 — 실사고 재현: 남이 양보한 자리를 나는 그대로 지날 수 있어야 한다.

    dg_02 가 7 에 서 있어 dg_03 이 양보했을 뿐인데, 무관한 dg_01 까지 7 을 못 가면
    목표가 줄줄이 '경로 없음'이 되어 순찰이 조기 종료된다.
    """
    engine, runner, _log = env
    runner._blacklist_add(engine.node_slot(7), "dg_03")     # dg_03 만의 사정

    mine = runner._plan_route(engine, 10, 8, set(), "dg_01")
    assert mine is not None and 7 in mine.nodes, "남의 양보에 내 최단경로를 빼앗겼다"

    theirs = runner._plan_route(engine, 10, 8, set(), "dg_03")
    assert theirs is not None and 7 not in theirs.nodes, "정작 양보한 본인이 안 피한다"


# ---------------------------- 불변식 5~6: 목표 지점 취급 ---------------------------- #
def test_target_is_not_abandoned_when_yielded(env):
    """불변식 5 — 목표 자리가 회피 목록에 있어도 목표 자체를 포기하지 않는다.

    자리 회피는 '지금 남이 서 있다'는 순간의 사정이고, 도착할 즈음엔 비어 있을 수 있다.
    정말 안 비면 _acquire_segment 가 대기했다 양보한다 — 판단은 거기서 하면 된다.
    (실사고: 지점 9 를 '지나가는' 경로는 찾으면서 9 를 '목표로' 삼는 것만 원천 차단됐다.)
    """
    engine, runner, _log = env
    runner._blacklist_add(engine.node_slot(8), "dg_01")     # 목표 8 자리가 회피 목록에

    route = runner._plan_route(engine, 10, 8, set(), "dg_01")
    assert route is not None, "목표가 회피 목록에 있다고 경로 자체를 포기했다"
    assert route.nodes[-1] == 8


def test_target_in_attempt_block_stays_blocked(env):
    """불변식 6 — 이번 시도에서 방금 못 간 자리로는 곧장 되돌아가지 않는다.

    attempt_block 까지 풀어 주면 '실패 → 같은 목표 재계획 → 또 실패'로 무한 왕복이 된다.
    """
    engine, runner, _log = env
    attempt_block = {engine.node_slot(8)}           # 방금 8 자리를 못 잡았다

    route = runner._plan_route(engine, 10, 8, attempt_block, "dg_01")
    assert route is None, "방금 실패한 목표로 곧장 되돌아갔다"


def test_passing_through_still_avoids_others_block(env):
    """불변식 5 의 경계 — 목표가 아닌 '경유' 지점은 회피가 그대로 살아 있어야 한다.

    목표만 예외로 푸는 것이지, 남이 서 있는 자리를 관통해도 된다는 뜻이 아니다.
    """
    engine, runner, _log = env
    runner._blacklist_add(engine.node_slot(7), "dg_01")     # 내가 7 을 피하기로 했다

    route = runner._plan_route(engine, 10, 8, set(), "dg_01")
    assert route is not None, "우회로가 있는데 경로를 못 찾았다"
    assert 7 not in route.nodes, f"경유 지점 회피가 풀렸다: {route.nodes}"
