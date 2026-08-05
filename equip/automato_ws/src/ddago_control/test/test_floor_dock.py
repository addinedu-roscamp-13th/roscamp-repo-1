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
    plan = (0.1, 0.10, 0.1)          # dist 0.10 ≥ CL_MIN_DIST → 기동 스킵 안 걸림(TURN1 검증)
    # 근접: d=0.20 → 계획 안 됨(후진 명령 v<0)
    fsm = F.DockFsm()
    fsm.state = 'CENTERLINE'
    v, w = fsm.update(True, 0.20, 0.0, 0.0, 0.0, 0, 0, plan=plan, odom_xy=(0, 0), n=99)
    assert fsm.cl_phase == 'PLAN'           # 아직 계획 안 잡힘
    # 신뢰거리 + 미정렬(yaw 10°): d=0.35 → 안정 N프레임 후 계획 잡힘 → TURN1
    fsm2 = F.DockFsm()
    fsm2.state = 'CENTERLINE'
    for _ in range(F.CL_PLAN_STABLE_N):     # 안정 확정(튄 프레임 방지)에 N프레임 필요
        fsm2.update(True, 0.35, 0.0, math.radians(10.0), 0.0, 0, 0, plan=plan, odom_xy=(0, 0), n=99)
    assert fsm2.cl_phase == 'TURN1'


def test_plan_aborts_when_yaw_too_large():
    """yaw 좁히기 정책 ③: 안정 yaw 가 YAW_NARROW_MAX 초과면 1회로도 못 좁힘 → 즉시 ABORT."""
    F.configure(D_STAGE=0.20)
    plan = (0.1, 0.10, 0.1)
    fsm = F.DockFsm()
    fsm.state = 'CENTERLINE'
    fsm.cl_phase = 'PLAN'
    for _ in range(F.CL_PLAN_STABLE_N):     # yaw 45°(임계 35° 초과) 안정 → 시도 없이 중단
        fsm.update(True, 0.30, 0.0, math.radians(45.0), 0.0, 0, 0, plan=plan, odom_xy=(0, 0), n=99)
    assert fsm.state == 'ABORT'


def test_plan_skips_maneuver_when_yaw_in_band():
    """yaw 좁히기 정책 ①: 안정 yaw 가 신뢰밴드(≤YAW_NARROW_OK)면 기동 없이 바로 ALIGN(진행)."""
    F.configure(D_STAGE=0.20)
    plan = (0.5, 0.10, -0.5)                 # th1 큰 값이라도 밴드면 기동 스킵돼야 함
    fsm = F.DockFsm()
    fsm.state = 'CENTERLINE'
    fsm.cl_phase = 'PLAN'
    for _ in range(F.CL_PLAN_STABLE_N):     # yaw 3°(밴드 8° 안) → 기동 없이 ALIGN
        fsm.update(True, 0.30, 0.0, math.radians(3.0), 0.0, 0, 0, plan=plan, odom_xy=(0, 0), n=99)
    assert fsm.state == 'ALIGN'
    assert fsm.cl_done is True


def test_plan_waits_for_stable_yaw():
    """PLAN 확정은 yaw 안정(좁은 범위) N프레임 필요 — yaw 가 ±플립하면 확정 보류(PLAN 유지)."""
    F.configure(D_STAGE=0.20)
    plan = (0.1, 0.10, 0.1)
    fsm = F.DockFsm()
    fsm.state = 'CENTERLINE'
    fsm.cl_phase = 'PLAN'
    for i in range(F.CL_PLAN_STABLE_N + 3):     # yaw 가 매 프레임 크게 튐(-40 ↔ +40)
        yaw = math.radians(40.0 if i % 2 else -40.0)
        fsm.update(True, 0.30, 0.0, yaw, 0.0, 0, 0, plan=plan, odom_xy=(0, 0), n=99)
    assert fsm.cl_phase == 'PLAN'               # 튐 지속 → 확정 보류(나쁜 프레임 커밋 방지)


def test_plan_backup_no_rotation():
    """(B) PLAN 근접(d<0.24) 재획득 후진은 회전 없이 직진(bearing 커도 w=0).

    근접·스큐 검출은 미약해서 후진과 '동시 회전'이 그걸 깨뜨려 마커를 놓쳤다.
    → 재획득 후진은 v<0, w=0(직진)만."""
    F.configure(D_STAGE=0.20)
    fsm = F.DockFsm()
    fsm.state = 'CENTERLINE'
    fsm.cl_phase = 'PLAN'
    v, w = fsm.update(True, 0.20, math.radians(10.0), 0.0, 0.0, 0, 0,
                      plan=(0.1, 0.05, 0.1), odom_xy=(0.0, 0.0), n=99)
    assert v < 0.0                          # 후진
    assert abs(w) < 1e-9                     # 회전 없음(bearing 10°인데도)


def test_plan_lost_marker_goes_to_search():
    """(A) PLAN 중 마커 상실이 LOST_TIMEOUT 넘으면 SEARCH(빠른 재탐색, 8초 대기 회피)."""
    fsm = F.DockFsm()
    fsm.state = 'CENTERLINE'
    fsm.cl_phase = 'PLAN'
    fsm.cl_plan_since = time.monotonic()
    fsm.lost_since = time.monotonic() - (F.LOST_TIMEOUT + 0.5)   # 이미 상실 타임아웃 경과
    fsm.update(False, 0, 0, 0, 0.0, 0, 0, odom_xy=(0.0, 0.0))
    assert fsm.state == 'SEARCH'
    assert fsm.cl_phase == 'PLAN'           # 재획득하면 다시 계획부터


def test_plan_lost_backs_straight_before_timeout():
    """(A) 상실 직후(타임아웃 전)엔 회전 없이 직진 후진으로 재획득 시도."""
    fsm = F.DockFsm()
    fsm.state = 'CENTERLINE'
    fsm.cl_phase = 'PLAN'
    v, w = fsm.update(False, 0, 0, 0, 0.0, 0, 0, odom_xy=(0.0, 0.0))
    assert fsm.state == 'CENTERLINE'
    assert v < 0.0 and abs(w) < 1e-9        # 직진 후진(회전 X)


def test_plan_reliable_window_gates():
    """로봇별 신뢰창(D_RELIABLE_MIN/MAX): 창 밖이면 계획 안 함.
    d>MAX → 접근(전진), 창 안 → 계획(TURN1). far 부정확 회피 + 헛후진 방지."""
    try:
        F.configure(D_STAGE=0.20, D_RELIABLE_MIN=0.20, D_RELIABLE_MAX=0.29)
        plan = (0.1, 0.10, 0.1)          # dist 0.10 ≥ CL_MIN_DIST → 기동 스킵 안 걸림(TURN1 검증)
        # d=0.40 > MAX(0.29): 계획 말고 전진 접근
        fsm = F.DockFsm(); fsm.state = 'CENTERLINE'; fsm.cl_phase = 'PLAN'
        v, w = fsm.update(True, 0.40, 0.0, math.radians(10.0), 0.0, 0, 0,
                          plan=plan, odom_xy=(0.0, 0.0), n=99)
        assert v > 0.0                       # 너무 멂 → 접근(전진)
        assert fsm.cl_phase == 'PLAN'         # 아직 계획 전
        # d=0.25 in [0.20,0.29]: 안정 N프레임 후 계획됨(미정렬 yaw10° → TURN1)
        fsm2 = F.DockFsm(); fsm2.state = 'CENTERLINE'
        for _ in range(F.CL_PLAN_STABLE_N):
            fsm2.update(True, 0.25, 0.0, math.radians(10.0), 0.0, 0, 0,
                        plan=plan, odom_xy=(0.0, 0.0), n=99)
        assert fsm2.cl_phase == 'TURN1'
    finally:
        F.configure(D_RELIABLE_MIN=0.24, D_RELIABLE_MAX=10.0)   # 기본 복원(교차오염 방지)


def test_centerline_backward_drive_when_target_behind():
    """근접(d<CL_TARGET_D, 중심선 위)이면 목표 G가 로봇 뒤 → 전진안은 th1≈±180° 왕복.
    후진안(dist<0) 자동선택으로 회전 최소(th1·th2≈0) → 헛턴 제거."""
    F.configure(D_STAGE=0.20, CL_TARGET_D=0.26, LATERAL_OFFSET=0.0)
    _d, _b, _y, plan, _gx, _gy = F.square_geometry((0.0, 0.24), 0.0)  # 정면·중심선, d=0.24<0.26
    th1, dist, th2 = plan
    assert dist < 0                              # 후진 선택
    assert abs(math.degrees(th1)) < 5            # 180° 왕복 아님
    assert abs(math.degrees(th2)) < 5


def test_centerline_forward_drive_when_target_ahead():
    """d>CL_TARGET_D면 목표 G가 앞 → 전진(dist>0), 회전 작음."""
    F.configure(D_STAGE=0.20, CL_TARGET_D=0.26, LATERAL_OFFSET=0.0)
    _d, _b, _y, plan, _gx, _gy = F.square_geometry((0.0, 0.30), 0.0)  # d=0.30>0.26
    th1, dist, th2 = plan
    assert dist > 0
    assert abs(math.degrees(th1)) < 5


def test_align_decelerates_near_stage():
    """ALIGN 접근이 목표(D_STAGE) 근처서 감속 → 오버슛(신뢰창 아래) 방지. 먼 데선 최대속."""
    F.configure(D_STAGE=0.20)
    fsm = F.DockFsm(); fsm.state = 'ALIGN'
    v_far, _ = fsm.update(True, 0.40, 0.0, 0.0, 0.0, 0, 0)     # 목표+decel_zone 밖 → 최대속
    fsm2 = F.DockFsm(); fsm2.state = 'ALIGN'
    v_near, _ = fsm2.update(True, 0.21, 0.0, 0.0, 0.0, 0, 0)   # 목표 바로 앞 → 감속
    assert abs(v_far - F.V_APPROACH) < 1e-9
    assert 0 < v_near < F.V_APPROACH
    assert v_near >= F.V_APPROACH * F.V_STAGE_MIN_FRAC - 1e-9  # 하한 이상


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
    """TURN2 후 VERIFY: 신뢰거리(d≥0.25)서 안정 N프레임 bearing/yaw 양호 → ALIGN(cl_done=True)."""
    F.configure(CL_VERIFY_D=0.25)
    fsm = F.DockFsm()
    fsm.state = 'CENTERLINE'
    fsm.cl_phase = 'VERIFY'
    for _ in range(F.CL_PLAN_STABLE_N):     # 안정 프레임 필요(튄 프레임 판정 방지)
        fsm.update(True, 0.30, 0.0, 0.0, 0.0, 0, 0, odom_xy=(0.0, 0.0))
    assert fsm.state == 'ALIGN'
    assert fsm.cl_done is True


def test_verify_aborts_when_still_skewed():
    """VERIFY(1회 좁힘 후): 안정 N프레임인데 yaw 신뢰밴드(YAW_NARROW_OK) 밖 → 재계획 없이 ABORT."""
    F.configure(CL_VERIFY_D=0.25)
    fsm = F.DockFsm()
    fsm.state = 'CENTERLINE'
    fsm.cl_phase = 'VERIFY'
    for _ in range(F.CL_PLAN_STABLE_N):     # yaw 20°(밴드 8° 밖)로 안정 → 좁힘 실패 판정
        fsm.update(True, 0.30, 0.0, math.radians(20.0), 0.0, 0, 0, odom_xy=(0.0, 0.0))
    assert fsm.state == 'ABORT'


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
    for _ in range(F.CL_PLAN_STABLE_N):     # 안정 N프레임 후 정렬 판단(median)
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
    """post_advance_m>0: REVERSE→HOLD→ADVANCE→DONE. ADVANCE 거리=rev_dist+margin
    (후진 되돌리기+여유) → 재획득 d≈D_STAGE+margin(신뢰창 안). 고정거리 아님."""
    F.configure(STAGE_SETTLE_SEC=0.0, POST_DOCK_HOLD_SEC=0.0, D_STAGE=0.20,
                D_RELIABLE_MAX=10.0)   # 창 상한 없음 → margin=post_advance_m
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
    fsm.update(False, 0, 0, 0, 0.0, 0, 0, odom_xy=(0.05, 0.0))   # 홀드 0초 → ADVANCE
    assert fsm.state == 'ADVANCE'
    assert abs(fsm.adv_target - (0.05 + 0.10)) < 1e-9           # rev_dist + margin = 0.15
    fsm.update(False, 0, 0, 0, 0.0, 0, 0, odom_xy=(0.05, 0.0))   # adv 기준점(adv_xy0=0.05)
    assert fsm.state == 'ADVANCE'
    fsm.update(False, 0, 0, 0, 0.0, 0, 0, odom_xy=(0.16, 0.0))   # 전진 0.11 < 0.15 → 아직 ADVANCE
    assert fsm.state == 'ADVANCE'
    fsm.update(False, 0, 0, 0, 0.0, 0, 0, odom_xy=(0.21, 0.0))   # 전진 0.16 ≥ 0.15 → DONE
    assert fsm.state == 'DONE'
    assert fsm.result_code == 0


def test_advance_target_clamped_to_window():
    """과도한 post_advance_m 이라도 margin 은 신뢰창 far 끝까지만 → 재획득 d≤MAX(창 밖 착지 방지)."""
    try:
        F.configure(POST_DOCK_HOLD_SEC=0.0, D_STAGE=0.20, D_RELIABLE_MAX=0.2875)
        fsm = F.DockFsm()
        fsm.post_advance_m = 0.30           # 과도 요청(고정 0.30 → 창 초과하던 값)
        fsm.rev_dist = 0.18
        fsm.state = 'HOLD'
        fsm.hold_start = 0.0
        fsm.update(False, 0, 0, 0, 0.0, 0, 0, odom_xy=(0.0, 0.0))   # HOLD→ADVANCE, adv_target 계산
        assert fsm.state == 'ADVANCE'
        # margin=min(0.30, MAX−D_STAGE=0.0875)=0.0875 → 재획득 d=D_STAGE+margin=0.2875=MAX(안 넘음)
        assert abs(fsm.adv_target - (0.18 + 0.0875)) < 1e-6
    finally:
        F.configure(D_RELIABLE_MAX=10.0)


def test_search_no_read_when_disabled():
    """read_enable=False(task_point_id 가 '1'~'3' 아님): H 검출 즉시 CENTERLINE(로마 인식 없음, 기존)."""
    fsm = F.DockFsm()
    fsm.update(True, 0.30, 0.0, 0.0, 0.0, 0, 0, plan=(0.1, 0.1, 0.1),
               odom_xy=(0, 0), n=99, read_enable=False)
    assert fsm.state == 'CENTERLINE'


def test_search_enters_read_when_enabled():
    """read_enable=True: SEARCH서 H 검출(경사 OK) → 정지하고 READ(로마 읽기) 진입(v=w=0)."""
    fsm = F.DockFsm()
    v, w = fsm.update(True, 0.30, 0.0, 0.0, 0.0, 0, 0, plan=(0.1, 0.1, 0.1),
                      odom_xy=(0, 0), n=99, read_enable=True, station_ok=False)
    assert fsm.state == 'READ'
    assert v == 0.0 and w == 0.0


def test_search_skips_read_when_too_oblique():
    """경사 과대(|yaw|>YAW_NARROW_MAX)면 READ 안 하고 계속 회전(부정확 카운트로 오매치 방지)."""
    fsm = F.DockFsm()
    v, w = fsm.update(True, 0.30, 0.0, math.radians(45.0), 0.0, 0, 0, plan=(0.1, 0.1, 0.1),
                      odom_xy=(0, 0), n=99, read_enable=True, station_ok=True)
    assert fsm.state == 'SEARCH'
    assert w == F.SEARCH_W                       # 회전 계속


def test_read_advances_when_station_matches():
    """READ서 station_ok=True + READ_HOLD 경과 → 목표 스테이션 확정 → CENTERLINE 진행."""
    fsm = F.DockFsm()
    fsm.state = 'READ'
    fsm.read_start = time.monotonic() - (F.READ_HOLD_SEC + 0.1)
    fsm.update(True, 0.30, 0.0, 0.0, 0.0, 0, 0, plan=(0.1, 0.1, 0.1),
               odom_xy=(0, 0), n=99, read_enable=True, station_ok=True)
    assert fsm.state == 'CENTERLINE'             # use_centerline=True 기본


def test_read_waits_while_station_mismatch():
    """READ서 station_ok=False면 READ_HOLD 지나도 진행 안 함(타임아웃 전까진 계속 읽음)."""
    fsm = F.DockFsm()
    fsm.state = 'READ'
    fsm.read_start = time.monotonic() - (F.READ_HOLD_SEC + 0.1)   # HOLD 는 지났지만
    fsm.update(True, 0.30, 0.0, 0.0, 0.0, 0, 0, plan=(0.1, 0.1, 0.1),
               odom_xy=(0, 0), n=99, read_enable=True, station_ok=False)
    assert fsm.state == 'READ'                   # 불일치 → 진행 보류


def test_read_timeout_rejects_wrong_station():
    """READ서 READ_TIMEOUT 넘게 목표ID 불일치 → 옆 스테이션 → SEARCH 복귀 + 쿨다운(지나침)."""
    fsm = F.DockFsm()
    fsm.state = 'READ'
    fsm.read_start = time.monotonic() - (F.READ_TIMEOUT + 0.1)
    v, w = fsm.update(True, 0.30, 0.0, 0.0, 0.0, 0, 0, plan=(0.1, 0.1, 0.1),
                      odom_xy=(0, 0), n=99, read_enable=True, station_ok=False)
    assert fsm.state == 'SEARCH'
    assert w == F.SEARCH_W                        # 회전해 지나침
    assert fsm.read_cooldown_until > time.monotonic()


def test_read_returns_to_search_when_marker_lost():
    """READ 중 H 놓치면(found=False) 재탐색(SEARCH)."""
    fsm = F.DockFsm()
    fsm.state = 'READ'
    fsm.read_start = time.monotonic()
    fsm.update(False, 0, 0, 0, 0.0, 0, 0, read_enable=True, station_ok=True)
    assert fsm.state == 'SEARCH'


def test_station_id_from_point():
    """task_point_id '1'~'3'→그 번호(로마 게이트 ON), 그 외/비숫자→0(게이트 OFF, 기존 도킹)."""
    from ddago_control.floor_dock import floor_detector as D
    assert D.station_id_from_point('1') == 1
    assert D.station_id_from_point('3') == 3
    assert D.station_id_from_point('CHARGE_01') == 0    # 예시 point → 게이트 off
    assert D.station_id_from_point('4') == 0            # ROMAN_MAX_ID 초과 → off
    assert D.station_id_from_point('') == 0


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
