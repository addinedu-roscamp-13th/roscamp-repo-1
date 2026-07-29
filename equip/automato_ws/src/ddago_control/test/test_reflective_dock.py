#!/usr/bin/env python3
"""반사테이프 후진 도킹 상태머신(reflective_fsm.ReflectiveDockFsm) 단위 테스트.

로봇·라이다 없이 FSM 로직만 검증한다. 관측(마커 pose, odom, 시각)을 가짜로
먹여 상태전이와 종료 결과를 확인한다. floor_dock 의 test_floor_dock.py 와 같은
방식(순수 모듈만 검증).

지키려는 것:
  * 기하(to_odom): 라이다→odom 변환이 라이다 오프셋·뒤보기(yaw=π)를 반영하는가.
  * 사전정렬: 이미 법선 위면 DRIVE 를 건너뛰고, 너무 멀면 ABORT(align_failed).
  * 후진: SWITCH_M 이하면 CREEP 로 넘어가고, odom 실거리로 CREEP 를 끝내는가(시간 아님).
  * 안전: 비상거리면 멈추고(tolerance), 마커를 잃으면 멈춘다(marker_not_found).
  * 제한시간: 마커 한 번도 못 보고 시간 초과 시 marker_not_found.

실행:
  source /opt/ros/jazzy/setup.bash
  cd equip/automato_ws && source install/setup.bash
  PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pytest src/ddago_control/test/test_reflective_dock.py -v
"""
import math

from ddago_control.reflective_dock import reflective_fsm as R


def _fsm():
    """기본 로봇값(rear=0.10, gap=0.02)의 새 FSM. target=0.12, emergency=0.105."""
    return R.ReflectiveDockFsm(rear_offset_m=0.10, stop_gap_m=0.02)


# ---------------------------------------------------------------- 기하 --- #
def test_norm_ang():
    assert abs(R.norm_ang(math.radians(190)) - math.radians(-170)) < 1e-9


def test_to_odom_rear_looking():
    """odom 원점·yaw0 에서 라이다 앞(+x) 점은, 라이다가 뒤를 보므로(π) odom -x 쪽."""
    fsm = _fsm()
    ox, oy, a = fsm.to_odom(0.30, 0.0, (0.0, 0.0, 0.0))
    assert abs(a - math.pi) < 1e-9              # 라이다 헤딩 = 로봇yaw + π
    assert ox < 0.0                             # 앞(+x)의 마커가 odom 에선 뒤(-x)
    assert abs(oy) < 1e-9


# ----------------------------------------------------- 사전정렬(SNAP) --- #
def test_snap_skips_drive_when_on_normal():
    """이미 법선 위(정면·정렬)면 이동거리~0 → DRIVE 건너뛰고 TURN2 로."""
    fsm = _fsm()
    odom = (0.0, 0.0, 0.0)
    for _ in range(R.SNAP_FRAMES):
        fsm.step((0.15, 0.0, 0.0), odom, now=0.0)   # 정면(y=0)·정렬(yaw=0)
    assert fsm.state == 'TURN2'                     # DRIVE 생략
    assert not fsm.done


def test_snap_aborts_when_too_far():
    """법선까지 이동이 MAX_MOVE_M(0.30) 초과면 스냅샷 이상 → align_failed 종료."""
    fsm = _fsm()
    odom = (0.0, 0.0, 0.0)
    for _ in range(R.SNAP_FRAMES):
        fsm.step((0.15, 0.40, 0.0), odom, now=0.0)  # 옆으로 크게 벗어남
    assert fsm.done
    assert fsm.result_code == R.RC_ALIGN_FAILED


# --------------------------------------------------- 후진(APPROACH) --- #
def test_approach_switches_to_creep():
    """SWITCH_M(0.18) 이하로 접근하면 CREEP 로 전환하고 남은 크립거리를 잡는다."""
    fsm = _fsm()
    fsm.state = 'APPROACH'
    v, w = fsm.step((0.15, 0.0, math.pi), (0.0, 0.0, 0.0), now=1.0)  # 정면, d=0.15
    assert fsm.state == 'CREEP'
    assert abs(fsm.creep_dist - (0.15 - fsm.target_m)) < 1e-6        # 0.15-0.12=0.03
    assert not fsm.done


def test_approach_emergency_stop():
    """비상거리(d<rear+0.005=0.105) 안으로 들어오면 즉시 정지(tolerance)."""
    fsm = _fsm()
    fsm.state = 'APPROACH'
    fsm.step((0.10, 0.0, math.pi), (0.0, 0.0, 0.0), now=1.0)         # d=0.10 < 0.105
    assert fsm.done
    assert fsm.result_code == R.RC_TOLERANCE


def test_approach_lost_marker():
    """접근 중 마커를 유예시간(0.7s) 넘게 잃으면 정지(marker_not_found)."""
    fsm = _fsm()
    fsm.state = 'APPROACH'
    fsm.last_good_t = 1.0                       # 마지막으로 마커 본 시각
    fsm.step(None, (0.0, 0.0, 0.0), now=2.0)    # 1.0s 경과(>0.7) 동안 마커 없음
    assert fsm.done
    assert fsm.result_code == R.RC_MARKER_NOT_FOUND


# ------------------------------------------------------ 후진(CREEP) --- #
def test_creep_finishes_on_odom_distance():
    """CREEP 는 odom 실이동거리로 끝난다(시간 아님) → 성공·gap=stop_gap."""
    fsm = _fsm()
    fsm.state = 'CREEP'
    fsm.creep_origin = (0.0, 0.0)
    fsm.creep_dist = 0.03
    fsm.step(None, (0.02, 0.0, 0.0), now=1.0)   # 2cm 이동 → 아직
    assert not fsm.done
    fsm.step(None, (0.03, 0.0, 0.0), now=1.1)   # 3cm 이동 → 완료
    assert fsm.done and fsm.success
    assert fsm.result_code == R.RC_OK
    assert abs(fsm.final_gap - 0.02) < 1e-9


# -------------------------------------------------------- 제한시간 --- #
def test_timeout_without_marker():
    """마커 한 번도 못 보고 MAX_SEC 초과 → marker_not_found."""
    fsm = _fsm()
    fsm.start_t = 0.0
    fsm.step(None, (0.0, 0.0, 0.0), now=R.MAX_SEC + 1.0)
    assert fsm.done
    assert fsm.result_code == R.RC_MARKER_NOT_FOUND


def test_configure_overrides_constant():
    """configure() 로 튜닝 상수를 덮어쓸 수 있고, 미지 키는 거부한다."""
    old = R.SWITCH_M
    try:
        R.configure(SWITCH_M=0.25)
        assert R.SWITCH_M == 0.25
    finally:
        R.configure(SWITCH_M=old)
    try:
        R.configure(NOPE=1.0)
        assert False, '미지 키를 거부해야 한다'
    except KeyError:
        pass
