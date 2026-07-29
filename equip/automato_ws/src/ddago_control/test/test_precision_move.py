#!/usr/bin/env python3
"""precision_move 순수 함수 단위 테스트 — ROS 없이 돈다.

정밀 주행의 계산이 틀리면 로봇이 **엉뚱한 방향으로 돌면서** 드러나므로, 현장에
나가기 전에 여기서 잡는 편이 훨씬 싸다. 실행:
  PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pytest src/ddago_control/test/test_precision_move.py -v
"""
import math

from ddago_control import precision_move as pm
import pytest


# --------------------------------------------------------------------------- #
# 각도 접기 — 먼 쪽으로 도는 사고를 막는 장치
# --------------------------------------------------------------------------- #
def test_normalize_angle_folds_to_short_way():
    """+270° 로 가라는 말은 -90° 로 돌라는 뜻이다."""
    assert pm.normalize_angle(math.radians(270)) == pytest.approx(
        math.radians(-90))
    assert pm.normalize_angle(math.radians(-190)) == pytest.approx(
        math.radians(170))
    assert pm.normalize_angle(0.5) == pytest.approx(0.5)


def test_quaternion_roundtrip():
    """yaw → 쿼터니언 → yaw 가 원래 값으로 돌아온다."""
    for deg in (-179.0, -90.0, 0.0, 1.0, 90.0, 179.0):
        yaw = math.radians(deg)
        z, w = pm.quaternion_from_yaw(yaw)
        assert pm.yaw_from_quaternion(0.0, 0.0, z, w) == pytest.approx(yaw)


# --------------------------------------------------------------------------- #
# 오차 분해 — 사진에 미치는 영향이 성분마다 다르다
# --------------------------------------------------------------------------- #
def test_pose_error_forward_component():
    """목표 헤딩(북) 기준으로 5cm 못 미쳐 서 있으면 fwd = +5cm."""
    target = (1.0, 1.0, math.pi / 2)          # 북쪽을 보고 서야 하는 지점
    cur = (1.0, 0.95, math.pi / 2)            # 남쪽으로 5cm 못 옴
    fwd, lat, dyaw = pm.pose_error(cur, target)
    assert fwd == pytest.approx(0.05)
    assert lat == pytest.approx(0.0, abs=1e-9)
    assert dyaw == pytest.approx(0.0, abs=1e-9)


def test_pose_error_lateral_component():
    """lat 은 '목표가 로봇 기준 왼쪽으로 얼마나 있는가'다.

    카메라가 왼쪽 90° 를 보므로, 로봇이 목표보다 오른쪽에 서 있으면(lat +) 그만큼
    피사체에서 멀어진 것이다. 부호가 뒤집히면 '멀다/가깝다'를 반대로 읽어 좌표를
    엉뚱한 쪽으로 옮기게 되므로 못을 박아 둔다.
    """
    target = (1.0, 1.0, math.pi / 2)          # 북쪽을 보고 서야 하는 지점
    cur = (1.03, 1.0, math.pi / 2)            # 동쪽 = 로봇의 오른쪽으로 3cm 치우침
    fwd, lat, _ = pm.pose_error(cur, target)
    assert fwd == pytest.approx(0.0, abs=1e-9)
    assert lat == pytest.approx(0.03)         # 피사체에서 3cm 멀다


def test_pose_error_yaw_is_folded():
    """각 오차도 짧은 쪽으로 접힌다.

    3.0rad 을 보는 로봇이 -3.0rad 을 봐야 한다면, 그냥 빼면 -6.0rad(-344°) 이라
    로봇이 먼 쪽으로 한 바퀴 가까이 돈다. 접으면 +0.28rad(+16°) — 짧은 쪽이다.
    """
    _, _, dyaw = pm.pose_error((0.0, 0.0, 3.0), (0.0, 0.0, -3.0))
    assert dyaw == pytest.approx(-6.0 + 2 * math.pi)
    assert abs(dyaw) < math.radians(20)


# --------------------------------------------------------------------------- #
# 이동 방향 — Nav2 에 넘길 도착 방향
# --------------------------------------------------------------------------- #
def test_travel_yaw_points_at_target():
    """정북으로 1m 가야 하면 +90°."""
    assert pm.travel_yaw((0.0, 0.0), (0.0, 1.0)) == pytest.approx(math.pi / 2)
    assert pm.travel_yaw((0.0, 0.0), (-1.0, 0.0)) == pytest.approx(math.pi)


def test_travel_yaw_none_for_short_move():
    """너무 짧은 이동은 None — 위치 노이즈가 그대로 방향 오차가 되기 때문."""
    assert pm.travel_yaw((0.0, 0.0), (0.02, 0.0), min_travel=0.10) is None
    assert pm.travel_yaw((0.0, 0.0), (0.20, 0.0), min_travel=0.10) is not None


# --------------------------------------------------------------------------- #
# 감속과 지나침 — 헌팅을 막는 두 장치
# --------------------------------------------------------------------------- #
def test_approach_speed_is_clamped_both_ends():
    """많이 남으면 상한, 거의 다 왔으면 하한(정지 마찰을 이길 최소 속도)."""
    assert pm.approach_speed(10.0, 2.0, 0.15, 0.45) == pytest.approx(0.45)
    assert pm.approach_speed(0.001, 2.0, 0.15, 0.45) == pytest.approx(0.15)
    assert pm.approach_speed(0.1, 2.0, 0.15, 0.45) == pytest.approx(0.2)


def test_approach_speed_ignores_sign():
    """크기만 돌려준다 — 방향은 호출부가 붙인다."""
    assert pm.approach_speed(-0.1, 2.0, 0.15, 0.45) == pytest.approx(0.2)


def test_overshot_detects_passing_the_target():
    """처음 돌던 방향과 남은 오차의 부호가 갈리면 지나친 것이다."""
    assert pm.overshot(-0.01, 1.0) is True     # +로 돌았는데 오차가 -
    assert pm.overshot(0.01, 1.0) is False     # 아직 덜 돌았다
    assert pm.overshot(0.01, -1.0) is True     # -로 돌았는데 오차가 +
