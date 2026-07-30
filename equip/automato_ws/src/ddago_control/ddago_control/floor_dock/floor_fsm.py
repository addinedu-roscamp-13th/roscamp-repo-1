# -*- coding: utf-8 -*-
"""바닥 H 마커 후진 도킹 상태머신 (순수 모듈, ROS 비의존).

RP-126. `floor_dock_ws/floor_pose.DockController`(ddago01 실주행 검증본)에서
FSM 만 이식. **자동 반복 도킹(테스트용)·웹·jog·print 는 제외** — 단일 도킹만 한다
(반복 재시도는 ACS 가 관장). 종료는 DONE(성공)/ABORT(실패) 종단 상태.

상태: SEARCH → CENTERLINE(turn-drive-turn 개루프 odom, 끝에 VERIFY 재정렬검증)
      → ALIGN/FACE → STAGED(정착) → TURN(180°) → REVERSE(동적, 벽 기준)
      → [post_advance_m>0 이면 HOLD(완료정지)→ADVANCE(후퇴)] → DONE

기하/튜닝 상수는 모듈 레벨(기본값). 노드가 `configure(**params)` 로 덮어쓴다.
"""
import math
import time

import numpy as np

# ── 탐색 ──
SEARCH_W = 0.2                    # SEARCH 회전 각속도 [rad/s]
SEARCH_REVS = 1.1                 # 최대 회전 바퀴수(초과 시 ABORT)
SEARCH_TIMEOUT = SEARCH_REVS * 2 * math.pi / SEARCH_W
LOST_TIMEOUT = 1.0                # 놓친 뒤 SEARCH 복귀까지 [s]
# ── 접근 / 중심선 정렬 ──
V_APPROACH = 0.05                 # 접근 전진 속도 [m/s]
D_STAGE = 0.20                    # 스테이징 거리(사각형 중심→base) [m]
LATERAL_OFFSET = 0.005            # 정렬 목표 횡 오프셋 [m] (+왼쪽)
BEARING_TOL = math.radians(3.0)   # 정면(FACE) 허용오차 [rad]
YAW_TOL = math.radians(5.0)       # 수직(중심선) 허용오차 [rad]
YAW_RELIABLE_D = 0.24             # yaw 신뢰 최소 거리 [m] (근접 FACE 완화 근거)
FACE_TIMEOUT = 6.0                # FACE 정렬 실패 판정 [s]
K_BEARING = 1.0                   # bearing → 각속도 게인
W_MAX = 0.5                       # 최대 각속도 [rad/s]
LOST_D_MARGIN = 0.05              # 이 여유거리 내에서 놓치면(정렬됨) STAGED [m]
N_STAGE_FLOOR = 8                 # (호환용) 코너 기반 조기 STAGED — H는 n=99라 미발동
STAGE_SETTLE_SEC = 0.6           # STAGED(auto) 후진 전 정착 대기 [s]. 정지한 채 d 여러 프레임
                                 #  모아 median(동적후진) 편차↓ + 모션블러↓ → 검출↑ (ddago01 검증)
# ── CENTERLINE ──
N_PLAN = 1                        # 계획 확정 최소 프레임(H는 검출 1장이면 계획)
CL_PLAN_TIMEOUT = 8.0
CL_PLAN_BACKUP_MAX = 0.15         # PLAN서 근접이면 후진 한계 [m]
# 중심선 기동 후 재정렬 검증(VERIFY): TURN2 직후 d 가 신뢰거리 아래라 근접 노이즈로 바로
#  STAGED 하면 삐뚤어진다 → 신뢰거리로 물러나 재확인, 벗어나면 재계획(수렴 반복). 놓치면 후진
#  재획득, 한도 넘으면 SEARCH(무한대기 방지). (ddago01 실주행 검증)
CL_VERIFY_D = 0.25               # 재확인 최소 거리 [m] (yaw 신뢰거리보다 살짝 위)
CL_MAX_REPLANS = 2               # 재계획 최대 횟수(초과 시 현재 정렬로 진행)
# 중심선 기동 목표점 거리 [m]. 스테이징(D_STAGE)이 아니라 신뢰거리(≥YAW_RELIABLE_D)에 두어,
#  기동이 끝난 자리서 VERIFY 가 후진 없이 바로 판정하게 한다(불필요한 후진 제거). ALIGN 이 이후
#  D_STAGE 까지 접근. 07-28: VERIFY 왕복 후진 제거 목적.
CL_TARGET_D = 0.26
# ── 180도 회전 ──
TURN_W = 0.25
TURN_TOL = math.radians(0.8)
K_TURN = 1.2
# ── 후진(odom 실거리 1:1, 벽 기준 동적) ──
V_REVERSE = 0.05
K_HEADING = 1.5                   # 후진 직진성 유지 게인(odom yaw)
DYNAMIC_REVERSE = True            # False 면 고정 REVERSE_DIST
WALL_GAP_TARGET = 0.030           # 목표 후면~벽 간격 [m] (07-28: 25→30 여유 상향, ddago01 검증)
REVERSE_K = -0.001                # 실측 상수 [m] (rear_to_wall = d + REVERSE_K − rev)
CROSSBAR_TO_WALL = 0.0425         # 가로바 중심→벽 [m] (표시 d 를 벽거리로 환산)
REV_MIN, REV_MAX = 0.02, 0.22     # 후진 안전 클램프 [m]
REVERSE_DIST = 0.070              # 고정모드/무검출 폴백
# ── 도킹 후 홀드·후퇴(반복 테스트용). post_advance_m>0 이면 REVERSE → HOLD → ADVANCE → DONE ──
POST_DOCK_HOLD_SEC = 2.5          # 반복 시 도킹 완료 후 정지 유지 [s] (완료 확인용)
V_ADVANCE = 0.05                  # 후퇴(전진) 속도 [m/s]

# 노드가 파라미터로 덮어쓸 수 있는 상수(configure)
_TUNABLE = (
    'D_STAGE', 'LATERAL_OFFSET', 'WALL_GAP_TARGET', 'REVERSE_K', 'CROSSBAR_TO_WALL',
    'DYNAMIC_REVERSE', 'REV_MIN', 'REV_MAX', 'REVERSE_DIST', 'V_APPROACH', 'V_REVERSE',
    'SEARCH_W', 'TURN_W', 'FACE_TIMEOUT', 'STAGE_SETTLE_SEC', 'CL_VERIFY_D',
    'CL_MAX_REPLANS', 'CL_TARGET_D', 'POST_DOCK_HOLD_SEC', 'V_ADVANCE',
)


def configure(**params):
    """노드 파라미터로 모듈 상수를 덮어쓴다(_TUNABLE 화이트리스트만)."""
    g = globals()
    for k, v in params.items():
        if k in _TUNABLE and v is not None:
            g[k] = v


def _clamp(x, lim):
    return max(-lim, min(lim, x))


def _ang_norm(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def square_geometry(center, heading):
    """마커(center_ground=(right,fwd)[m], heading[rad]) -> 도킹 기하 일괄.

    반환 (d, bearing, yaw, (th1, dist, th2), gx, gy). 로봇 프레임 x=전방, y=좌.
    접근축은 '로봇이 보는 방향(beta)에 가까운 축'을 고르고(마커 90° 대칭), 목표를
    LATERAL_OFFSET 만큼 로봇 왼쪽으로 이동."""
    Sr, Sf = float(center[0]), float(center[1])
    Cx, Cy = Sf, -Sr
    beta = math.atan2(Cy, Cx)
    a0 = _ang_norm(-heading)

    def _toward(a):
        return min((_ang_norm(a), _ang_norm(a + math.pi)),
                   key=lambda x: abs(_ang_norm(x - beta)))
    axes = (_toward(a0), _toward(a0 + math.pi / 2))
    face_dir = min(axes, key=lambda x: abs(_ang_norm(x - beta)))
    if LATERAL_OFFSET:
        Cx += -LATERAL_OFFSET * math.sin(face_dir)   # 로봇프레임 왼쪽 (-sin, cos)
        Cy += LATERAL_OFFSET * math.cos(face_dir)
        Sr, Sf = -Cy, Cx
    d = math.hypot(Sr, Sf)
    bearing = math.atan2(Sr, Sf)
    yaw = _ang_norm(-face_dir)
    N = _ang_norm(face_dir + math.pi)
    gx = Cx + CL_TARGET_D * math.cos(N)   # 중심선 기동 목표 = 신뢰거리(ALIGN 이 D_STAGE 까지 접근)
    gy = Cy + CL_TARGET_D * math.sin(N)
    th1 = math.atan2(gy, gx)
    dist = math.hypot(gx, gy)
    th2 = _ang_norm(face_dir - th1)
    return d, bearing, yaw, (th1, dist, th2), gx, gy


def docking_values(center, heading):
    """마커 -> (d, bearing, yaw). d=카메라 바닥투영점→가로바 중심 수평거리(오프셋 반영)."""
    d, bearing, yaw, _, _, _ = square_geometry(center, heading)
    return d, bearing, yaw


def centerline_plan(center, heading):
    """중심선 스테이징점 G까지 turn-drive-turn 경로. 반환 (th1, dist, th2, gx, gy)."""
    _, _, _, plan, gx, gy = square_geometry(center, heading)
    return plan[0], plan[1], plan[2], gx, gy


class DockFsm:
    """바닥 H 후진 도킹 FSM. update(...) -> (v, w). 종료=DONE/ABORT.

    노드가 매 프레임 검출값(found,d,bearing,yaw,plan)+odom 을 넣고 (v,w)를 받아 cmd_vel 발행.
    auto=True(액션서버 기본)면 STAGED 에서 대기 없이 회전+후진 진행. use_centerline=True.
    """

    def __init__(self):
        self.state = "SEARCH"
        self.lost_since = None
        self.search_start = None
        self.turn_target = None
        self.reverse_start = None
        self.rev_xy0 = None
        self.rev_dist = REVERSE_DIST
        self.staged_d = []            # 정지구간(FACE/STAGED) d 누적(중앙값 → 동적 후진)
        self.staged_since = None      # STAGED 진입 시각(정착 대기 STAGE_SETTLE_SEC)
        self.hold_yaw = None
        self.last_d = None
        self.last_bearing = 99.0
        self.last_yaw = 99.0
        self.yaw_verified = False
        self.cl_done = False
        self.face_start = None
        self.auto = True              # 액션서버: STAGED 자동 진행
        self.use_centerline = True
        self.cl_phase = "PLAN"
        self.cl_odom0 = None
        self.cl_th1 = 0.0
        self.cl_dist = 0.0
        self.cl_th2 = 0.0
        self.cl_drive_start = None
        self.cl_xy0 = None
        self.cl_plan_since = None
        self.cl_plan_xy0 = None
        self.cl_plan = None           # (th1_deg, dist_cm, th2_deg) 로깅용
        self.cl_replans = 0           # VERIFY 재계획 횟수(한도 CL_MAX_REPLANS)
        self.cl_verify_since = None   # VERIFY 재획득 시작시각(무한대기 방지)
        self.cl_verify_xy0 = None
        self.post_advance_m = 0.0     # >0 이면 REVERSE 완료 후 HOLD→전진(반복 테스트용) → DONE
        self.hold_start = None        # HOLD(도킹완료 정지) 시작시각
        self.adv_xy0 = None
        self.obstacle_ahead = False   # 노드가 라이다로 세팅. ADVANCE 중 True면 조기 완료(정지)
        self.obstacle_behind = False  # 노드가 세팅. PLAN 후진 중 True면 후진 중지 → ALIGN 폴백
        self.reverse_odom_used = None  # REVERSE 완료 시: odom 실거리(True)/시간폴백(False)
        self.reverse_trav = 0.0        # REVERSE 실이동거리 [m] (계측)
        self.note = ""                # 이상/폴백 사유 + 동적후진 요약
        self.result_code = 0          # 0 성공 /1 미검출 /3 중단 /4 정렬실패
        self.final_gap = 0.0          # 예상 후면~벽 갭 [m] (REVERSE 확정 시)

    def _abort(self, why, code=3):
        self.state = "ABORT"
        self.note = why
        self.result_code = code

    def _warn(self, why):
        self.note = why

    def _reverse_dist(self):
        """동적 후진거리: STAGED d(중앙값)로 후면이 목표 벽갭에 서도록 계산 + 안전 클램프."""
        if not DYNAMIC_REVERSE:
            return REVERSE_DIST
        ds = self.staged_d if self.staged_d else ([self.last_d] if self.last_d else [])
        if not ds:
            self._warn('STAGED d 없음 — 고정 REVERSE_DIST 폴백')
            return REVERSE_DIST
        dref = float(np.median(ds))
        rev = dref + REVERSE_K - WALL_GAP_TARGET
        rev_c = min(REV_MAX, max(REV_MIN, rev))
        self.final_gap = dref + REVERSE_K - rev_c
        clamped = ' [clamp]' if abs(rev_c - rev) > 1e-6 else ''
        self.note = ('dyn-reverse d=%.1fcm(n=%d) rev=%.0fmm exp_gap=%.0fmm%s'
                     % (dref * 100, len(ds), rev_c * 1000, self.final_gap * 1000, clamped))
        return rev_c

    def update(self, found, d, bearing, yaw, odom_yaw, gx, gy, proceed=True, n=99,
               plan=None, odom_xy=None, n_plan=None):
        now = time.monotonic()
        if n_plan is None:
            n_plan = N_PLAN

        if self.state in ("DONE", "ABORT"):
            return 0.0, 0.0

        # 신뢰거리서 yaw 가 좋았는지 한 번 기록(근접 FACE 완화 근거)
        if found and d > YAW_RELIABLE_D and abs(yaw) < YAW_TOL:
            self.yaw_verified = True

        # ── 블라인드 기동(odom) ──
        if self.state == "TURN":
            if odom_yaw is None:
                return 0.0, 0.0
            err = _ang_norm(self.turn_target - odom_yaw)
            if abs(err) < TURN_TOL:
                self.state = "REVERSE"
                self.reverse_start = now
                self.rev_xy0 = odom_xy
                self.hold_yaw = odom_yaw
                return -V_REVERSE, 0.0
            return 0.0, _clamp(K_TURN * err, TURN_W)

        if self.state == "REVERSE":
            odom_used = odom_xy is not None and self.rev_xy0 is not None
            if odom_used:
                trav = math.hypot(odom_xy[0] - self.rev_xy0[0],
                                  odom_xy[1] - self.rev_xy0[1])
                done = trav >= self.rev_dist
            else:
                trav = (now - self.reverse_start) * V_REVERSE   # 시간기반 추정거리
                done = now - self.reverse_start >= self.rev_dist / V_REVERSE
            if done:
                self.reverse_odom_used = odom_used   # 계측: odom 실거리 vs 시간폴백
                self.reverse_trav = trav
                self.result_code = 0
                if self.post_advance_m > 0.0:        # 반복 테스트: 완료 정지(HOLD) 후 후퇴 → DONE
                    self.state = "HOLD"
                    self.hold_start = now
                else:
                    self.state = "DONE"              # 단일 도킹 완료(벽에 붙어 유지)
                return 0.0, 0.0
            w = 0.0
            if odom_yaw is not None and self.hold_yaw is not None:
                w = _clamp(-K_HEADING * _ang_norm(odom_yaw - self.hold_yaw), W_MAX)
            return -V_REVERSE, w

        if self.state == "HOLD":                 # 반복: 도킹 완료 후 POST_DOCK_HOLD_SEC 정지 유지
            if now - self.hold_start >= POST_DOCK_HOLD_SEC:
                self.state = "ADVANCE"
                self.adv_xy0 = None
            return 0.0, 0.0

        if self.state == "ADVANCE":              # 도킹 후 전진(반복용). odom 실거리, 없으면 생략
            if self.adv_xy0 is None:
                self.adv_xy0 = odom_xy
            if odom_xy is None or self.adv_xy0 is None:
                self.state = "DONE"
                return 0.0, 0.0
            trav = math.hypot(odom_xy[0] - self.adv_xy0[0], odom_xy[1] - self.adv_xy0[1])
            # 목표 도달 or 전방 장애물(노드가 obstacle_ahead 세팅) → 조기 정지·완료(ABORT 아님, 다음 진행)
            if trav >= self.post_advance_m or self.obstacle_ahead:
                if self.obstacle_ahead:
                    self._warn('ADVANCE 중 전방 장애물 — %.0fmm 전진 후 정지·완료' % (trav * 1000))
                self.state = "DONE"
                return 0.0, 0.0
            return V_ADVANCE, 0.0

        # ── 스테이징 대기 ──
        if self.state == "STAGED":
            if self.staged_since is None:
                self.staged_since = now
            if found and d > 0.0 and len(self.staged_d) < 60:
                self.staged_d.append(float(d))   # 정지 중 d 누적(동적 후진 중앙값)
            # auto: 정착 대기(STAGE_SETTLE_SEC) 동안 정지한 채 d 여러 프레임 모은 뒤 진행
            #  (n=1 단일샘플 → median 무효 방지). 수동은 proceed 즉시.
            go = (now - self.staged_since >= STAGE_SETTLE_SEC) if self.auto else proceed
            if go:
                if odom_yaw is None:
                    self._abort('odom 없음 — 180도 회전 불가 (bringup 미실행?)')
                    return 0.0, 0.0
                self.rev_dist = self._reverse_dist()
                self.state = "TURN"
                self.turn_target = _ang_norm(odom_yaw + math.pi)
            return 0.0, 0.0

        # ── 중심선 정렬 기동(turn-drive-turn 개루프 odom) ──
        if self.state == "CENTERLINE":
            if self.cl_phase == "PLAN":
                if self.cl_plan_since is None:
                    self.cl_plan_since = now
                    self.cl_plan_xy0 = odom_xy
                # 신뢰거리(d≥YAW_RELIABLE_D)서 좋은 프레임일 때만 계획
                if not (found and plan is not None and n >= n_plan
                        and d >= YAW_RELIABLE_D):
                    backed = 0.0
                    if odom_xy is not None and self.cl_plan_xy0 is not None:
                        backed = math.hypot(odom_xy[0] - self.cl_plan_xy0[0],
                                            odom_xy[1] - self.cl_plan_xy0[1])
                    # (A) 마커 완전 상실: 회전 없이 잠깐 직진 후진(화각 확대)으로 재획득 시도,
                    #     LOST_TIMEOUT 넘게 못 잡으면 SEARCH(제자리 회전 탐색)로 — 8초 대기 회피.
                    if not found:
                        self.lost_since = self.lost_since or now
                        if now - self.lost_since > LOST_TIMEOUT:
                            self._warn('PLAN 마커 상실 %.1fs — SEARCH 재탐색'
                                       % (now - self.lost_since))
                            self.state = "SEARCH"
                            self.search_start = None
                            self.cl_phase = "PLAN"
                            self.cl_plan_since = None
                            self.cl_plan_xy0 = None
                            self.lost_since = None
                            return 0.0, 0.0
                        # 후방 여유 있으면 직진 후진(회전 X)으로 재획득, 막혔으면 정지 대기.
                        if (not self.obstacle_behind and backed < CL_PLAN_BACKUP_MAX
                                and odom_xy is not None):
                            return -V_APPROACH, 0.0
                        return 0.0, 0.0
                    # 여기부턴 found=True(근접 d<YAW_RELIABLE_D 또는 부실 프레임).
                    self.lost_since = None
                    # 후방 장애물/시간초과/후진한계면 더 못 물러남 → ALIGN 폴백(충돌 방지, 다음 진행).
                    if (self.obstacle_behind or now - self.cl_plan_since > CL_PLAN_TIMEOUT
                            or backed >= CL_PLAN_BACKUP_MAX):
                        why = ('후방 장애물' if self.obstacle_behind
                               else '%.0fcm 후진/시간초과' % (backed * 100))
                        self._warn('중심선 계획 불가(%s) — ALIGN 접근 폴백' % why)
                        self.state = "ALIGN"       # 계획 실패 → 접근 폴백(cl_done=False)
                        return 0.0, 0.0
                    # (B) 신뢰거리까지 직진 후진만(회전 제거) — 근접 검출을 동시 회전이 깨뜨리지 않게.
                    if d < D_STAGE + CL_PLAN_BACKUP_MAX and odom_xy is not None:
                        return -V_APPROACH, 0.0
                    return 0.0, 0.0
                if odom_yaw is None:
                    self._abort('odom 없음 — 중심선 기동 불가 (bringup 미실행?)')
                    return 0.0, 0.0
                self.cl_th1, self.cl_dist, self.cl_th2 = plan
                # 이미 정렬(신뢰거리서 bearing·yaw 작음)이면 turn-drive-turn 스킵 → 바로 ALIGN.
                if abs(bearing) < BEARING_TOL and abs(yaw) < YAW_TOL:
                    self._warn('이미 정렬됨 — 중심선 기동 스킵 → ALIGN')
                    self.state = "ALIGN"
                    self.cl_done = True
                    self.lost_since = None
                    return 0.0, 0.0
                self.cl_odom0 = odom_yaw
                self.cl_plan = (math.degrees(self.cl_th1), self.cl_dist * 100,
                                math.degrees(self.cl_th2))
                self.cl_phase = "TURN1"
                return 0.0, 0.0
            if odom_yaw is None:
                self._abort('odom 끊김 — 중심선 기동 중단 (%s)' % self.cl_phase)
                return 0.0, 0.0
            if self.cl_phase == "TURN1":
                err = _ang_norm(self.cl_odom0 + self.cl_th1 - odom_yaw)
                if abs(err) < TURN_TOL:
                    self.cl_phase = "DRIVE"
                    self.cl_drive_start = now
                    self.cl_xy0 = odom_xy
                    self.hold_yaw = odom_yaw
                    return 0.0, 0.0
                return 0.0, _clamp(K_TURN * err, TURN_W)
            if self.cl_phase == "DRIVE":
                if odom_xy is not None and self.cl_xy0 is not None:
                    trav = math.hypot(odom_xy[0] - self.cl_xy0[0],
                                      odom_xy[1] - self.cl_xy0[1])
                    done = trav >= self.cl_dist
                else:
                    done = now - self.cl_drive_start >= self.cl_dist / V_APPROACH
                if done:
                    self.cl_phase = "TURN2"
                    return 0.0, 0.0
                w = _clamp(-K_HEADING * _ang_norm(odom_yaw - self.hold_yaw), W_MAX)
                return V_APPROACH, w
            if self.cl_phase == "TURN2":
                err = _ang_norm(self.cl_odom0 + self.cl_th1 + self.cl_th2 - odom_yaw)
                if abs(err) < TURN_TOL:
                    self.cl_phase = "VERIFY"   # 신뢰거리 재확인 → OK면 ALIGN, 벗어나면 재계획
                    self.lost_since = None
                    return 0.0, 0.0
                return 0.0, _clamp(K_TURN * err, TURN_W)
            if self.cl_phase == "VERIFY":
                # 기동이 신뢰거리(CL_TARGET_D≥YAW_RELIABLE_D)에서 끝났으므로 ★후진 없이★ 그 자리서
                #  판정. 미검출/근접(yaw 불신)이면 계획을 신뢰하고 진행(불필요한 후진 제거, 07-28).
                if (not found) or d < YAW_RELIABLE_D:
                    self.state = "ALIGN"
                    self.cl_done = True
                    self.lost_since = None
                    return 0.0, 0.0
                if abs(bearing) < BEARING_TOL and abs(yaw) < YAW_TOL:
                    self.state = "ALIGN"
                    self.cl_done = True                  # 검증 통과 → 근접 yaw 신뢰(FACE)
                    self.lost_since = None
                    return 0.0, 0.0
                if self.cl_replans < CL_MAX_REPLANS:
                    self.cl_replans += 1
                    self._warn('중심선 재정렬 %d/%d: b=%.1f y=%.1f 벗어남 → 재계획'
                               % (self.cl_replans, CL_MAX_REPLANS,
                                  math.degrees(bearing), math.degrees(yaw)))
                    self.cl_phase = "PLAN"               # 물러난 포즈서 다시 계획
                    self.cl_plan_since = None
                    self.cl_plan_xy0 = None
                    return 0.0, 0.0
                self._warn('중심선 재계획 한도(%d) 초과 — 현재 정렬로 진행' % CL_MAX_REPLANS)
                self.state = "ALIGN"
                self.cl_done = True
                self.lost_since = None
                return 0.0, 0.0
            return 0.0, 0.0

        # ── 탐색 ──
        if self.state == "SEARCH":
            if found:
                self.state = "CENTERLINE" if self.use_centerline else "ALIGN"
                self.cl_phase = "PLAN"
                self.cl_plan_since = None
                self.cl_plan_xy0 = None
                self.cl_replans = 0
                self.cl_verify_since = None
                self.lost_since = None
                self.search_start = None
            else:
                if self.search_start is None:
                    self.search_start = now
                if now - self.search_start > SEARCH_TIMEOUT:
                    self._abort('탐색 실패 — %.0f초 회전했는데 마커 미검출' % SEARCH_TIMEOUT,
                                code=1)
                    return 0.0, 0.0
                return 0.0, SEARCH_W

        # ── ALIGN/FACE 중 놓치면 ──
        if not found:
            if (self.state in ("ALIGN", "FACE") and self.last_d is not None
                    and self.last_d < D_STAGE + LOST_D_MARGIN
                    and abs(self.last_bearing) < BEARING_TOL
                    and (abs(self.last_yaw) < YAW_TOL or self.yaw_verified)):
                self.state = "STAGED"
                return 0.0, 0.0
            self.lost_since = self.lost_since or now
            if now - self.lost_since > LOST_TIMEOUT:
                self.state = "SEARCH"
                self.search_start = None
            return 0.0, 0.0
        self.lost_since = None
        self.last_d = d
        self.last_bearing = bearing
        self.last_yaw = yaw

        w_align = _clamp(-K_BEARING * bearing, W_MAX)   # bearing 추종(중앙 유지)

        if self.state == "ALIGN":
            aligned = abs(bearing) < BEARING_TOL and abs(yaw) < YAW_TOL
            if d <= D_STAGE or (n < N_STAGE_FLOOR and aligned):
                self.state = "FACE"
                self.face_start = now
                return 0.0, w_align
            return V_APPROACH, w_align

        if self.state == "FACE":
            # FACE 는 v=0(제자리) → d 는 이미 staging 거리. 후진용 d 를 미리 모아둔다
            #  (근접서 STAGED 진입 후 보드 놓쳐 n=1 되던 것 방지, 정지구간이라 편향 없음).
            if found and d > 0.0 and len(self.staged_d) < 60:
                self.staged_d.append(float(d))
            # 근접선 비전 yaw 불신 → CENTERLINE 완주(cl_done)면 bearing 만으로 STAGED
            yaw_ok = (abs(yaw) < YAW_TOL or self.cl_done
                      or (d < YAW_RELIABLE_D and self.yaw_verified))
            if abs(bearing) < BEARING_TOL and yaw_ok:
                self.state = "STAGED"
                return 0.0, 0.0
            if self.face_start is not None and now - self.face_start > FACE_TIMEOUT:
                self._abort('정면 정렬 실패 — %.0f초 내 bearing<%.0f° yaw<%.0f° 미달'
                            % (FACE_TIMEOUT, math.degrees(BEARING_TOL),
                               math.degrees(YAW_TOL)), code=4)
                return 0.0, 0.0
            return 0.0, w_align

        return 0.0, 0.0
