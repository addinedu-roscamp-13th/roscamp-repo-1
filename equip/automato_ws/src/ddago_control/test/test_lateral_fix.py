#!/usr/bin/env python3
"""정밀 조준의 횡(lat) 보정 단위 테스트 — navigate_server._lateral_fix.

로봇 없이 검증한다. 오차(_error)와 라이다 여유(_scan_clearance)를 가짜로 꽂고,
실제로 내려간 동작(_nudge_ang / _nudge_lin)을 기록해 순서·부호·가드를 본다.

지키려는 것:
  * 차동구동은 옆으로 못 가므로 '돌아서 → 가서 → 되돌아' 3단계로 옮긴다.
  * 부호: lat 이 +면(목표가 왼쪽) 왼쪽으로 돌고, -면 오른쪽으로 돈다.
  * 가드 셋 — 꺼져 있으면 / 너무 작으면 / 너무 크면 / 주변이 좁으면 하지 않는다.
    특히 마지막은 실측 근거가 있다: 충전소 진입 노드 wp22 는 벽까지 8cm 라
    그 자리에서 90° 를 돌면 닿는다(wp24 는 24cm 로 넉넉).

실행:
  source /opt/ros/jazzy/setup.bash
  cd equip/automato_ws && source install/setup.bash
  PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pytest src/ddago_control/test/test_lateral_fix.py -v
"""
import math

from ddago_control.navigate_server import NavigateServer
import pytest
import rclpy
from rclpy.parameter import Parameter


@pytest.fixture
def srv():
    """정밀주행 켜진 서버 하나. 실제 주행은 안 하고 계산만 쓴다."""
    rclpy.init()
    node = NavigateServer(parameter_overrides=[
        Parameter('precision_enable', value=True),
        Parameter('refine_lateral', value=True),
    ])
    node._moves = []                                  # 내려간 동작 기록
    node._nudge_ang = lambda a: node._moves.append(('ang', a))
    node._nudge_lin = lambda d: node._moves.append(('lin', d))
    yield node
    node.destroy_node()
    rclpy.shutdown()


def _set(node, lat, clear=1.0):
    """오차(fwd, lat, yaw)와 라이다 여유를 꽂는다."""
    node._error = lambda _t: (0.0, lat, 0.0)
    node._scan_clearance = lambda: clear


def test_moves_sideways_in_three_steps(srv):
    """정상 범위의 옆 오차는 회전 → 전진 → 역회전으로 없앤다."""
    _set(srv, lat=+0.04)
    srv._lateral_fix((0.0, 0.0, 0.0), wid=24)
    kinds = [m[0] for m in srv._moves]
    assert kinds == ['ang', 'lin', 'ang']
    # 목표가 왼쪽(+)이면 왼쪽으로 돌고, 간 만큼 되돌아온다.
    assert srv._moves[0][1] == pytest.approx(math.pi / 2)
    assert srv._moves[1][1] == pytest.approx(0.04)
    assert srv._moves[2][1] == pytest.approx(-math.pi / 2)


def test_direction_follows_sign(srv):
    """옆 오차가 -면 반대로 돈다(오른쪽)."""
    _set(srv, lat=-0.05)
    srv._lateral_fix((0.0, 0.0, 0.0), wid=24)
    assert srv._moves[0][1] == pytest.approx(-math.pi / 2)
    assert srv._moves[1][1] == pytest.approx(0.05)      # 거리는 항상 양수


def test_skips_when_disabled(srv):
    """꺼져 있으면 아무것도 하지 않는다(기본값이 꺼짐이다)."""
    srv.set_parameters([Parameter('refine_lateral', value=False)])
    _set(srv, lat=+0.04)
    srv._lateral_fix((0.0, 0.0, 0.0), wid=24)
    assert srv._moves == []


def test_skips_small_error(srv):
    """임계보다 작으면 건드리지 않는다 — 괜히 회전 2회를 넣지 않는다."""
    _set(srv, lat=+0.01)
    srv._lateral_fix((0.0, 0.0, 0.0), wid=24)
    assert srv._moves == []


def test_skips_huge_error(srv):
    """한계보다 크면 손대지 않는다 — 그 정도면 경로·좌표 쪽 문제다."""
    _set(srv, lat=+0.20)
    srv._lateral_fix((0.0, 0.0, 0.0), wid=24)
    assert srv._moves == []


def test_skips_when_no_room_to_turn(srv):
    """주변이 좁으면 하지 않는다 — 회전하다 닿는 것이 오차보다 나쁘다.

    wp22(CHARGE_01) 실측 재현: 벽까지 8cm 라 90° 회전이 불가능하다.
    """
    _set(srv, lat=+0.04, clear=0.08)
    srv._lateral_fix((0.0, 0.0, 0.0), wid=22)
    assert srv._moves == []


def test_runs_when_room_is_enough(srv):
    """여유가 충분하면 수행한다(wp24 실측 24cm)."""
    _set(srv, lat=+0.04, clear=0.24)
    srv._lateral_fix((0.0, 0.0, 0.0), wid=24)
    assert [m[0] for m in srv._moves] == ['ang', 'lin', 'ang']


def test_runs_when_scan_missing(srv):
    """라이다를 아직 못 받았으면(None) 여유 판정을 건너뛰고 수행한다.

    스캔이 없다고 보정을 막으면, 라이다가 잠깐 끊긴 것만으로 도킹 자세가 나빠진다.
    """
    _set(srv, lat=+0.04, clear=None)
    srv._lateral_fix((0.0, 0.0, 0.0), wid=24)
    assert [m[0] for m in srv._moves] == ['ang', 'lin', 'ang']


def test_reports_whether_it_moved(srv):
    """수행했으면 True, 건너뛰었으면 False 를 돌려준다 — 호출부가 이걸 본다."""
    _set(srv, lat=+0.04)
    assert srv._lateral_fix((0.0, 0.0, 0.0), wid=24) is True
    _set(srv, lat=+0.01)                                  # 임계 미만 → 생략
    assert srv._lateral_fix((0.0, 0.0, 0.0), wid=24) is False
    srv.set_parameters([Parameter('refine_lateral', value=False)])
    _set(srv, lat=+0.04)
    assert srv._lateral_fix((0.0, 0.0, 0.0), wid=24) is False


def test_disabled_refine_adds_no_extra_rotation(srv):
    """★ 회귀: 횡보정이 꺼져 있으면 _refine 의 회전 횟수가 도입 전과 같아야 한다.

    2026-08-03 현장 사고. 횡보정을 넣으면서 _refine 끝에 마무리 회전을 조건 없이
    붙였더니, 꺼져 있는데도 회전이 1회 늘었다. _nudge_ang 은 목표를 지나치면
    되돌리지 않고 멈추는데(그게 헌팅 방지책이다) 그 방지는 **한 번의 호출 안**에서만
    성립한다 → 호출을 하나 더 붙이자 지나친 만큼을 반대로 되돌리는 코드가 되어
    wp5 에서 좌우 미세 진동이 났고, 벽 쪽으로 밀려 서면서 다음 지점(wp6)으로 가는
    경로가 안 나와 촬영까지 통째로 건너뛰었다.

    '동작은 종전과 같다'를 주석이 아니라 테스트로 지킨다.
    """
    srv.set_parameters([Parameter('refine_lateral', value=False)])

    class _WP:                       # _refine 이 보는 필드만 갖춘 가짜 waypoint
        waypoint_id, x, y, yaw = 5, 0.66, -0.018, 3.11

    # 방향이 늘 tolerance 밖으로 남아 있는 상태 = 오버슈트가 반복되는 최악 조건.
    srv._error = lambda _t: (0.0, 0.0, math.radians(3.0))
    srv._scan_clearance = lambda: 1.0
    srv._refine(_WP())

    rounds = srv._refine_rounds
    # for 루프에서 회전 rounds 회 + 전후진 0 회(fwd 오차 0) + 루프 뒤 마무리 회전 1 회.
    # 횡보정이 꺼져 있으므로 그 뒤로는 아무것도 더 돌지 않는다.
    assert [m[0] for m in srv._moves] == ['ang'] * (rounds + 1)
