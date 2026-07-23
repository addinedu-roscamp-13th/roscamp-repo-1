#!/usr/bin/env python3
"""RP-102  E4-6 도킹 상태머신(DockFsm) 단위 테스트.

로봇·카메라 없이 상태머신 로직만 검증한다. DockFsm 은 ROS 와 분리되어 있어
(관측·odom 을 인자로 받고 (v, w) 만 돌려준다) 가짜 값을 먹여 전 구간을 돌릴 수 있다.

여기서 지키려는 것:
  * CENTERING 이 turn-drive-turn 을 계획대로 수행하는가 — 이 기동이 off-axis 배치
    대응의 핵심이라, 회귀하면 "중심선에서 벗어나 배치하면 실패"로 되돌아간다.
  * DRIVE/REVERSE 가 **시간이 아니라 odom 실이동거리**로 끝나는가 — 시간 기반은
    실제 속도가 명령값보다 느린 만큼 짧게 가 중심선에 못 미친다(현장에서 3cm 부족).
  * 정렬 못 하면 삐뚤게 붙이지 않고 멈추는가(FACE_TIMEOUT) — 안전장치.

실행:
  source /opt/ros/jazzy/setup.bash
  cd equip/automato_ws && source install/setup.bash
  PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pytest src/ddago_control/test/test_dock.py -v
"""
import math

from ddago_control.dock_server import (
    RC_ALIGN_FAILED, RC_MARKER_NOT_FOUND, RC_OK, DockFsm,
)
import pytest

STAGING = 0.24
REVERSE = 0.15
DT = 0.1                      # 가짜 적분 간격 [s]


def make_cfg():
    """dock_server 의 현장 튜닝값과 같은 조합(DockServer._cfg 가 만드는 dict)."""
    cfg = {
        'v_approach': 0.05, 'v_reverse': 0.05, 'search_w': 0.2, 'turn_w': 0.25,
        'turn_tol': math.radians(0.8), 'k_turn': 1.2, 'k_bearing': 1.0,
        'k_heading': 1.5, 'w_max': 0.5,
        'bearing_tol': math.radians(3.0), 'yaw_tol': math.radians(5.0),
        'face_timeout': 6.0, 'lost_timeout': 1.0, 'plan_timeout': 8.0,
        'n_plan': 10, 'n_stage_floor': 8,
    }
    cfg['search_timeout'] = 1.1 * 2 * math.pi / cfg['search_w']
    return cfg


def make_fsm():
    return DockFsm(make_cfg(), STAGING, REVERSE)


# 관측: (d, bearing, yaw, corners)
FAR_OFF_AXIS = (0.40, math.radians(10), math.radians(30), 20)
ALIGNED_NEAR = (0.23, math.radians(0.5), math.radians(1.0), 15)
# 계획: (th1[rad], dist[m], th2[rad])
PLAN = (math.radians(25), 0.15, math.radians(-60))


def _spin_turn(fsm, yaw, t0, until, odom_xy=(0.0, 0.0), obs=None, plan=None):
    """제자리 회전을 odom 에 적분해가며 until(fsm) 이 참이 될 때까지 돌린다."""
    for i in range(800):
        _v, w = fsm.update(t0 + i * DT, obs is not None, obs, plan, yaw, odom_xy)
        if until(fsm):
            return yaw
        yaw += w * DT
    pytest.fail('회전이 끝나지 않음 (phase=%s)' % fsm.phase)


def _spin_drive(fsm, yaw, x, t0, until, obs=None, plan=None):
    """직진을 odom 위치에 적분해가며 until(fsm) 이 참이 될 때까지 돌린다."""
    for i in range(800):
        v, _w = fsm.update(t0 + i * DT, obs is not None, obs, plan, yaw, (x, 0.0))
        if until(fsm):
            return x
        x += v * DT
    pytest.fail('직진이 끝나지 않음 (phase=%s)' % fsm.phase)


def test_search_timeout_reports_marker_not_found():
    """마커를 못 찾으면 제자리 회전하다가 정해진 바퀴수에서 실패로 끝난다.

    무한 회전을 막는 안전장치라, 여기가 깨지면 로봇이 계속 돈다.
    """
    fsm = make_fsm()
    _v, w = fsm.update(0.0, False, None, None, 0.0, (0.0, 0.0))
    assert fsm.phase == 'SEARCHING'
    assert w > 0, '탐색 중에는 회전 명령이 나가야 한다'

    fsm.update(0.0 + make_cfg()['search_timeout'] + 1.0, False, None, None,
               0.0, (0.0, 0.0))
    assert fsm.done
    assert fsm.result_code == RC_MARKER_NOT_FOUND


def test_centering_plan_is_executed_by_odom():
    """CENTERING 이 계획한 회전·직진·회전을 odom 기준으로 수행한다.

    DRIVE 는 **시간이 아니라 실이동거리**로 끝나야 한다. 시간 기반이면 실제 속도가
    명령값보다 느린 만큼 짧게 가 중심선에 못 미친다.
    """
    fsm = make_fsm()
    fsm.update(0.0, True, FAR_OFF_AXIS, PLAN, 0.0, (0.0, 0.0))
    assert fsm.phase == 'CENTERING'

    # PLAN 단계: 좋은 프레임(코너 충분)이면 계획을 확정하고 TURN1 로 넘어간다.
    fsm.update(0.1, True, FAR_OFF_AXIS, PLAN, 0.0, (0.0, 0.0))
    assert fsm._cl == 'TURN1'

    yaw = _spin_turn(fsm, 0.0, 1.0, lambda f: f._cl != 'TURN1',
                     obs=FAR_OFF_AXIS, plan=PLAN)
    assert yaw == pytest.approx(PLAN[0], abs=math.radians(1.5)), \
        'TURN1 이 계획 각도만큼 돌지 않았다'

    x = _spin_drive(fsm, yaw, 0.0, 40.0, lambda f: f._cl != 'DRIVE',
                    obs=FAR_OFF_AXIS, plan=PLAN)
    assert x == pytest.approx(PLAN[1], abs=0.02), \
        'DRIVE 가 odom 실이동거리로 끝나지 않았다'

    yaw = _spin_turn(fsm, yaw, 80.0, lambda f: f.phase != 'CENTERING',
                     odom_xy=(x, 0.0), obs=FAR_OFF_AXIS, plan=PLAN)
    assert fsm.phase == 'APPROACHING'
    assert yaw == pytest.approx(PLAN[0] + PLAN[2], abs=math.radians(1.5)), \
        'TURN2 가 계획 각도만큼 돌지 않았다'


def test_full_sequence_reaches_done_with_odom_reverse():
    """정렬된 근거리 관측이면 STAGED → 180도 회전 → 후진까지 완주한다.

    후진도 odom 실이동거리로 끝나야 한다(갭이 명령값과 1:1 로 대응해야 튜닝이 된다).
    """
    fsm = make_fsm()
    fsm.phase = 'APPROACHING'

    # d <= staging 이라 미세정렬로 들어가고, 정렬 조건을 만족하면 STAGED.
    fsm.update(0.0, True, ALIGNED_NEAR, None, 0.0, (0.0, 0.0))
    fsm.update(0.1, True, ALIGNED_NEAR, None, 0.0, (0.0, 0.0))
    assert fsm.phase == 'STAGED'

    # STAGED 는 곧바로 180도 회전 목표를 잡는다.
    fsm.update(1.0, True, ALIGNED_NEAR, None, 0.0, (0.0, 0.0))
    assert fsm.phase == 'ROTATING'

    yaw = _spin_turn(fsm, 0.0, 2.0, lambda f: f.phase != 'ROTATING')
    assert fsm.phase == 'REVERSING'
    assert abs(_ang(yaw)) == pytest.approx(math.pi, abs=math.radians(1.5)), \
        '180도를 돌지 않았다 — 스큐로 직결된다'

    x = 0.0
    for i in range(800):
        v, _w = fsm.update(200.0 + i * DT, False, None, None, yaw, (x, 0.0))
        if fsm.done:
            break
        x += v * DT                       # v < 0 이므로 뒤로 간다
    assert fsm.done
    assert fsm.result_code == RC_OK
    assert abs(x) == pytest.approx(REVERSE, abs=0.02), \
        '후진이 odom 실이동거리로 끝나지 않았다'


def test_residual_yaw_aborts_instead_of_docking_crooked():
    """정렬 못 하면 삐뚤게 붙이지 않고 멈춘다(FACE_TIMEOUT).

    중심선 이탈이 남아 있으면 bearing 은 0 이어도 yaw 가 남는다. 그대로 후진하면
    스큐가 그대로 도크에 박히므로, 시간 안에 못 맞추면 실패로 끝내야 한다.
    """
    fsm = make_fsm()
    fsm.phase = 'APPROACHING'
    bad = (0.23, math.radians(0.2), math.radians(10.0), 15)   # bearing OK, yaw 초과

    fsm.update(0.0, True, bad, None, 0.0, (0.0, 0.0))
    assert not fsm.done, '허용시간 안에는 계속 맞춰본다'

    fsm.update(make_cfg()['face_timeout'] + 1.0, True, bad, None, 0.0, (0.0, 0.0))
    assert fsm.done
    assert fsm.result_code == RC_ALIGN_FAILED


def test_lost_marker_far_returns_to_search():
    """스테이징 근처가 아닌 곳에서 마커를 놓치면 재탐색으로 돌아간다."""
    fsm = make_fsm()
    fsm.phase = 'APPROACHING'
    fsm.update(0.0, True, (0.50, 0.0, 0.0, 20), None, 0.0, (0.0, 0.0))

    fsm.update(0.1, False, None, None, 0.0, (0.0, 0.0))
    assert fsm.phase == 'APPROACHING', '놓친 직후에는 잠깐 기다린다'

    fsm.update(0.1 + make_cfg()['lost_timeout'] + 0.5, False, None, None,
               0.0, (0.0, 0.0))
    assert fsm.phase == 'SEARCHING'


def _ang(a):
    return (a + math.pi) % (2 * math.pi) - math.pi
