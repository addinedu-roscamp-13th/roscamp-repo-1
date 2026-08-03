#!/usr/bin/env python3
"""반사마커 pose 계산(marker_detector.compute_pose) 단위 테스트.

라이다·로봇 없이 '점 몇 개로 만든 코너'를 직접 먹여 위치·방향을 확인한다.

지키려는 것:
  * 꼭짓점: 두 면 직선의 교점이 코너 좌표로 나온다.
  * **법선 방향**: 마커가 로봇을 향하는 쪽으로 나온다 — 어떤 이유로 반대로
    계산되든 되돌려진다.
    (2026-08-03 실사고: 마커 정면 축에서 6.6cm 옆에 선 채 도킹하니 법선이 180°
     뒤집혀 로봇이 마커를 등지는 대신 마주 본 채 후진 단계에 들어갔다 → β=-179°.
     비스듬히 보면 한 면이 짧게 잘려 꼭짓점이 반대편에 잡히는 것이 원인이다.)
    근거는 기하가 아니라 물리다: 반사테이프는 벽에 붙어 있고 라이다는 그 앞에
    있으므로, 법선이 라이다 반대편을 가리키는 일은 있을 수 없다.

실행:
  source /opt/ros/jazzy/setup.bash
  cd equip/automato_ws && source install/setup.bash
  PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pytest src/ddago_control/test/test_marker_pose.py -v
"""
import math

from ddago_control.reflective_dock import marker_detector as M


def _pt(x, y):
    """compute_pose 는 x·y 만 본다(각도·거리·밝기는 앞 단계에서만 쓰인다)."""
    return M.Point(angle_deg=0.0, range_m=math.hypot(x, y), intensity=50.0, x=x, y=y)


def _face(vx, vy, dx, dy, n=6, step=0.015):
    """꼭짓점 (vx,vy) 에서 (dx,dy) 방향으로 뻗는 면 하나(점 n개)."""
    u = M._unit(dx, dy)
    return [_pt(vx + u[0] * step * i, vy + u[1] * step * i) for i in range(n)]


def test_pose_normal_points_at_robot():
    """면이 로봇 반대쪽으로 뻗은 정상 코너 → 법선이 로봇(원점)을 향한다."""
    vx, vy = 0.30, 0.0                      # 코너는 로봇 앞 30cm
    marker = {
        "face_a": _face(vx, vy, +1.0, +1.0),   # 꼭짓점에서 뒤쪽(+x)으로 벌어짐
        "face_b": _face(vx, vy, +1.0, -1.0),
    }
    pose = M.compute_pose(marker)
    assert pose is not None
    assert abs(pose["x"] - vx) < 1e-6 and abs(pose["y"] - vy) < 1e-6
    # 로봇은 원점이므로 마커에서 로봇으로 가는 방향은 -x (=π).
    assert abs(M._norm_ang(pose["yaw_rad"] - math.pi)) < 1e-6


def test_pose_flipped_corner_is_corrected():
    """면이 로봇 쪽으로 뻗어 법선이 뒤집혀 나오는 코너 → 180° 되돌려진다.

    비스듬히 봐서 꼭짓점이 반대편에 잡힌 상황을 좌표로 재현한 것이다.
    수정 전에는 이 입력이 yaw=0(로봇 반대편)을 내놓았고, 그 값으로 도킹
    사전정렬이 로봇을 정반대로 돌려세웠다.
    """
    vx, vy = 0.30, 0.0
    marker = {
        "face_a": _face(vx, vy, -1.0, +1.0),   # 꼭짓점에서 로봇쪽(-x)으로 벌어짐
        "face_b": _face(vx, vy, -1.0, -1.0),
    }
    pose = M.compute_pose(marker)
    assert pose is not None
    # 검증이 없으면 yaw=0(로봇 반대편)이 나온다. 되돌려져 π 가 되어야 한다.
    assert abs(M._norm_ang(pose["yaw_rad"] - math.pi)) < 1e-6


def test_pose_correction_survives_oblique_view():
    """옆에서 비스듬히 본 코너에서도 법선은 로봇 쪽 반평면 안에 있다.

    각도까지 정확히 맞히지는 못해도(면이 잘려 보이므로) **앞뒤는 틀리지 않는다**는
    것이 이 검증의 목적이다. 도킹 실패를 가른 것이 정확히 그 앞뒤였다.
    """
    vx, vy = 0.25, 0.20                     # 로봇 정면 축에서 옆으로 벗어난 코너
    marker = {
        "face_a": _face(vx, vy, -1.0, +0.2),   # 뒤집혀 계산되기 쉬운 배치
        "face_b": _face(vx, vy, -0.2, -1.0),
    }
    pose = M.compute_pose(marker)
    assert pose is not None
    to_robot = math.atan2(-vy, -vx)
    assert abs(M._norm_ang(pose["yaw_rad"] - to_robot)) <= math.pi / 2.0


def test_pose_none_when_faces_parallel():
    """두 면이 평행하면 교점이 없어 pose 를 만들지 않는다(기존 동작 유지)."""
    marker = {
        "face_a": _face(0.30, +0.05, 0.0, 1.0),
        "face_b": _face(0.30, -0.05, 0.0, 1.0),
    }
    assert M.compute_pose(marker) is None
