#!/usr/bin/env python3
"""RP-126 바닥 H 도킹 상태머신(floor_fsm.DockFsm) 단위 테스트.

로봇·카메라 없이 FSM 로직만 검증한다. 관측(found,d,bearing,yaw,plan)+odom 을
가짜로 먹여 전 구간을 돌린다.

지키려는 것:
  * 기하(square_geometry): 정면 마커 → bearing~0, 목표 G 가 중심선 위, 횡오프셋 반영.
  * 동적 후진(_reverse_dist): STAGED d(중앙값) → 벽 기준 목표 갭에 서도록 rev 계산 + 클램프.
  * STAGED→TURN(180°)→REVERSE(odom 실거리)→DONE 이 odom 으로 끝나는가(시간 아님).
  * CENTERLINE PLAN 은 신뢰거리(d≥YAW_RELIABLE_D)서만 잡는가(근접 부실계획 방지).
  * 정렬 못 하면 삐뚤 붙이지 않고 ABORT 하는가(FACE_TIMEOUT, 안전).

실행:
  source /opt/ros/jazzy/setup.bash
  cd equip/automato_ws && source install/setup.bash
  PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pytest src/ddago_control/test/test_floor_dock.py -v
"""
import math
import time

from ddago_control.floor_dock import floor_fsm as F

DT = 0.05


def test_geometry_centered():
    """정면 마커(0, 0.4) heading 0 → bearing~0, gx=0.4-CL_TARGET_D, 횡오프셋 5mm 반영."""
    F.configure(D_STAGE=0.20, LATERAL_OFFSET=0.005, CL_TARGET_D=0.26)
    d, b, y, plan, gx, gy = F.square_geometry((0.0, 0.40), 0.0)
    assert abs(d - 0.40) < 0.02
    assert abs(gx - 0.14) < 0.01            # 중심선 기동 목표 = 중심 앞 CL_TARGET_D(0.26)
    assert abs(gy - 0.005) < 0.002          # 왼쪽 5mm


def test_geometry_offset_sign():
    """오른쪽에 놓인 마커 → bearing 양수(+ = 오른쪽)."""
    F.configure(LATERAL_OFFSET=0.0)
    _, b, *_ = F.square_geometry((0.10, 0.40), 0.0)
    assert b > 0


def test_reverse_dist_wall_gap():
    """STAGED d=0.20, wall_gap 0.030 → rear_to_wall = d + K − rev = 목표 갭."""
    F.configure(WALL_GAP_TARGET=0.030, REVERSE_K=-0.001, REV_MIN=0.02, REV_MAX=0.22,
                DYNAMIC_REVERSE=True)
    fsm = F.DockFsm()
    fsm.staged_d = [0.20, 0.20, 0.205]
    rev = fsm._reverse_dist()
    dref = 0.20
    assert abs((dref + F.REVERSE_K - rev) - 0.030) < 1e-6   # 예상 갭 = 목표
    assert F.REV_MIN <= rev <= F.REV_MAX
    assert abs(fsm.final_gap - 0.030) < 1e-6


def test_reverse_clamp_bad_d():
    """튄 d(0.40)라도 클램프로 과도 후진 방지."""
    F.configure(WALL_GAP_TARGET=0.025, REVERSE_K=-0.001, REV_MIN=0.02, REV_MAX=0.22)
    fsm = F.DockFsm()
    fsm.staged_d = [0.40]
    rev = fsm._reverse_dist()
    assert rev == F.REV_MAX                 # 0.40−0.001−0.025=0.374 → 클램프 0.22


def test_staged_to_turn_to_reverse_to_done():
    """STAGED→TURN(180° odom)→REVERSE(odom 실거리)→DONE. 시간 아닌 odom 으로 종료."""
    F.configure(WALL_GAP_TARGET=0.025, REVERSE_K=-0.001, REV_MIN=0.02, REV_MAX=0.22,
                STAGE_SETTLE_SEC=0.0)   # 테스트선 정착 대기 없이 즉시 진행
    fsm = F.DockFsm()
    fsm.state = 'STAGED'
    fsm.staged_d = [0.20, 0.20, 0.20]
    fsm.auto = True
    oy, ox = 0.0, (0.0, 0.0)
    # STAGED: auto → rev_dist 확정 후 TURN
    fsm.update(False, 0, 0, 0, oy, 0, 0, odom_xy=ox)
    assert fsm.state == 'TURN'
    target = fsm.turn_target
    # TURN: w 를 적분해 odom_yaw 를 target 으로
    for _ in range(2000):
        v, w = fsm.update(False, 0, 0, 0, oy, 0, 0, odom_xy=ox)
        oy = F._ang_norm(oy + w * DT)
        if fsm.state == 'REVERSE':
            break
    assert fsm.state == 'REVERSE'
    assert abs(F._ang_norm(oy - target)) < math.radians(2)   # 180° 회전 완료
    # REVERSE: v(<0) 를 적분해 odom_xy 이동거리 rev_dist 도달 → DONE
    rev = fsm.rev_dist
    for _ in range(2000):
        v, w = fsm.update(False, 0, 0, 0, oy, 0, 0, odom_xy=ox)
        ox = (ox[0] + v * math.cos(oy) * DT, ox[1] + v * math.sin(oy) * DT)
        if fsm.state == 'DONE':
            break
    assert fsm.state == 'DONE'
    assert fsm.result_code == 0             # RC_OK
    trav = math.hypot(ox[0] - fsm.rev_xy0[0], ox[1] - fsm.rev_xy0[1])
    assert abs(trav - rev) < 0.02           # odom 실이동거리로 종료(시간 아님)


def test_centerline_plan_only_at_reliable_distance():
    """근접(d<0.24)선 PLAN 안 잡고 후진 시도, 신뢰거리(d≥0.24)면 PLAN→TURN1."""
    F.configure(D_STAGE=0.20)
    plan = (0.1, 0.05, 0.1)
    # 근접: d=0.20 → 계획 안 됨(후진 명령 v<0)
    fsm = F.DockFsm()
    fsm.state = 'CENTERLINE'
    v, w = fsm.update(True, 0.20, 0.0, 0.0, 0.0, 0, 0, plan=plan, odom_xy=(0, 0), n=99)
    assert fsm.cl_phase == 'PLAN'           # 아직 계획 안 잡힘
    # 신뢰거리 + 미정렬(yaw 10°): d=0.35 → 계획 잡힘 → TURN1
    fsm2 = F.DockFsm()
    fsm2.state = 'CENTERLINE'
    fsm2.update(True, 0.35, 0.0, math.radians(10.0), 0.0, 0, 0, plan=plan, odom_xy=(0, 0), n=99)
    assert fsm2.cl_phase == 'TURN1'


def test_face_timeout_aborts_when_unaligned():
    """CENTERLINE 미완주(cl_done=False)에 yaw 안 맞으면 FACE_TIMEOUT 후 ABORT."""
    F.configure(FACE_TIMEOUT=0.05)
    fsm = F.DockFsm()
    fsm.state = 'FACE'
    fsm.cl_done = False
    fsm.yaw_verified = False
    fsm.face_start = time.monotonic() - 1.0   # 이미 타임아웃 경과
    # bearing 은 맞지만 yaw 크게 틀어짐(근접이지만 yaw_verified 없음) → ABORT
    fsm.update(True, 0.19, math.radians(0.5), math.radians(15.0), 0.0, 0, 0)
    assert fsm.state == 'ABORT'
    assert fsm.result_code == 4             # RC_ALIGN_FAILED


def test_face_trusts_centerline():
    """CENTERLINE 완주(cl_done=True)면 근접 yaw 쓰레기라도 bearing 만으로 STAGED."""
    fsm = F.DockFsm()
    fsm.state = 'FACE'
    fsm.cl_done = True
    fsm.face_start = time.monotonic()
    fsm.update(True, 0.19, math.radians(0.5), math.radians(15.0), 0.0, 0, 0)
    assert fsm.state == 'STAGED'


def test_verify_passes_to_align_when_aligned():
    """TURN2 후 VERIFY: 신뢰거리(d≥0.25)서 bearing/yaw 양호 → ALIGN(cl_done=True)."""
    F.configure(CL_VERIFY_D=0.25)
    fsm = F.DockFsm()
    fsm.state = 'CENTERLINE'
    fsm.cl_phase = 'VERIFY'
    fsm.update(True, 0.30, 0.0, 0.0, 0.0, 0, 0, odom_xy=(0.0, 0.0))
    assert fsm.state == 'ALIGN'
    assert fsm.cl_done is True


def test_verify_replans_when_skewed():
    """VERIFY: 신뢰거리서 yaw 벗어남 → 재계획(PLAN), cl_replans 증가(수렴 반복)."""
    F.configure(CL_VERIFY_D=0.25, CL_MAX_REPLANS=2)
    fsm = F.DockFsm()
    fsm.state = 'CENTERLINE'
    fsm.cl_phase = 'VERIFY'
    fsm.update(True, 0.30, 0.0, math.radians(10.0), 0.0, 0, 0, odom_xy=(0.0, 0.0))
    assert fsm.cl_phase == 'PLAN'
    assert fsm.cl_replans == 1
    assert fsm.state == 'CENTERLINE'


def test_verify_trusts_plan_when_not_found():
    """VERIFY: 미검출이면 ★후진 없이★ 계획 신뢰 → ALIGN(cl_done). (불필요한 후진 제거)"""
    fsm = F.DockFsm()
    fsm.state = 'CENTERLINE'
    fsm.cl_phase = 'VERIFY'
    v, w = fsm.update(False, 0, 0, 0, 0.0, 0, 0, odom_xy=(0.0, 0.0))
    assert fsm.state == 'ALIGN'
    assert fsm.cl_done is True
    assert v == 0.0 and w == 0.0            # 후진 안 함


def test_centerline_skip_when_aligned():
    """PLAN서 이미 정렬(신뢰거리 bearing/yaw 작음)이면 turn-drive-turn 스킵 → ALIGN(cl_done)."""
    F.configure(D_STAGE=0.20)
    fsm = F.DockFsm()
    fsm.state = 'CENTERLINE'
    fsm.cl_phase = 'PLAN'
    fsm.update(True, 0.30, 0.0, 0.0, 0.0, 0, 0, plan=(0.1, 0.05, 0.1), odom_xy=(0, 0), n=99)
    assert fsm.state == 'ALIGN'
    assert fsm.cl_done is True


def test_plan_align_fallback_when_obstacle_behind():
    """PLAN 후진 중 후방 장애물(obstacle_behind)이면 후진 중지 → ALIGN 폴백(충돌 방지)."""
    fsm = F.DockFsm()
    fsm.state = 'CENTERLINE'
    fsm.cl_phase = 'PLAN'
    fsm.obstacle_behind = True
    # 근접(d<신뢰거리)이라 계획 못 잡는 상황 + 후방 장애물 → ALIGN 폴백
    fsm.update(True, 0.20, 0.0, 0.0, 0.0, 0, 0, plan=(0.1, 0.05, 0.1), odom_xy=(0, 0), n=99)
    assert fsm.state == 'ALIGN'


def test_advance_obstacle_early_stop():
    """ADVANCE 중 obstacle_ahead=True 면 목표 전에 정지·완료(DONE), ABORT 아님."""
    F.configure(STAGE_SETTLE_SEC=0.0, POST_DOCK_HOLD_SEC=0.0)
    fsm = F.DockFsm()
    fsm.post_advance_m = 0.30
    fsm.state = 'ADVANCE'
    fsm.adv_xy0 = (0.0, 0.0)
    fsm.obstacle_ahead = True
    fsm.update(False, 0, 0, 0, 0.0, 0, 0, odom_xy=(0.10, 0.0))   # 0.10<0.30 이지만 장애물 → DONE
    assert fsm.state == 'DONE'
    assert fsm.result_code == 0


def test_post_advance_after_reverse():
    """post_advance_m>0 이면 REVERSE 완료 → HOLD → ADVANCE(odom 전진) → DONE."""
    F.configure(STAGE_SETTLE_SEC=0.0, POST_DOCK_HOLD_SEC=0.0)   # 테스트선 홀드 0
    fsm = F.DockFsm()
    fsm.post_advance_m = 0.10
    fsm.state = 'REVERSE'
    fsm.rev_dist = 0.05
    fsm.rev_xy0 = (0.0, 0.0)
    fsm.reverse_start = 0.0
    fsm.hold_yaw = 0.0
    fsm.update(False, 0, 0, 0, 0.0, 0, 0, odom_xy=(0.05, 0.0))   # 후진완료 → HOLD
    assert fsm.state == 'HOLD'
    assert fsm.reverse_odom_used is True
    fsm.update(False, 0, 0, 0, 0.0, 0, 0, odom_xy=(0.05, 0.0))   # 홀드 0초 경과 → ADVANCE
    assert fsm.state == 'ADVANCE'
    fsm.update(False, 0, 0, 0, 0.0, 0, 0, odom_xy=(0.05, 0.0))   # adv 기준점 잡고 전진
    assert fsm.state == 'ADVANCE'
    fsm.update(False, 0, 0, 0, 0.0, 0, 0, odom_xy=(0.16, 0.0))   # 전진 0.11≥0.10 → DONE
    assert fsm.state == 'DONE'
    assert fsm.result_code == 0


def test_single_dock_no_hold_no_advance():
    """post_advance_m=0(실배포)이면 REVERSE 완료 즉시 DONE(HOLD/ADVANCE 없음)."""
    F.configure(STAGE_SETTLE_SEC=0.0)
    fsm = F.DockFsm()               # post_advance_m 기본 0.0
    fsm.state = 'REVERSE'
    fsm.rev_dist = 0.05
    fsm.rev_xy0 = (0.0, 0.0)
    fsm.reverse_start = 0.0
    fsm.hold_yaw = 0.0
    fsm.update(False, 0, 0, 0, 0.0, 0, 0, odom_xy=(0.05, 0.0))
    assert fsm.state == 'DONE'
    assert fsm.result_code == 0
