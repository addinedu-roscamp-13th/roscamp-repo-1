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
# SEARCH 중 H 검출 시 정지하고 로마 숫자(스테이션 ID) 읽기 (READ). read_enable=True 일 때만.
#  goal.task_point_id 가 '1'~'3' 이면 노드가 read_enable=True + station_ok(목표ID 일치)를 넣는다.
READ_HOLD_SEC = 1.0               # H 검출 시 정지하고 로마 숫자 읽는 최소 시간 [s]
READ_TIMEOUT = 2.5                # 읽기 최대 — 합의 실패(옆 스테이션)면 재탐색 [s]
READ_COOLDOWN = 2.0               # 옆 스테이션 거부 후 재READ 금지(회전해 지나침) [s]
# ── 접근 / 중심선 정렬 ──
V_APPROACH = 0.05                 # 접근 전진 속도 [m/s]
V_DECEL_ZONE = 0.05               # STAGED 목표 앞 이 거리부터 감속 [m] (오버슛 방지)
V_STAGE_MIN_FRAC = 0.3            # 감속 하한(V_APPROACH 대비, 너무 느리면 못 감)
D_STAGE = 0.20                    # 스테이징 거리(사각형 중심→base) [m]
LATERAL_OFFSET = 0.005            # 정렬 목표 횡 오프셋 [m] (+왼쪽)
BEARING_TOL = math.radians(3.0)   # 정면(FACE) 허용오차 [rad]
YAW_TOL = math.radians(5.0)       # 수직(중심선) 허용오차 [rad]
YAW_RELIABLE_D = 0.24             # yaw 신뢰 최소 거리 [m] (근접 FACE 완화 근거)
# 거리(측정) 신뢰 창 [m] — 이 창에서만 중심선 계획. 노드가 로봇별 floor_dock_cal(dw)로 덮는다.
#  창 밖: d<MIN 너무 가까움(직진 후진), d>MAX 너무 멂(접근). 기본은 하위호환(하한=yaw신뢰, 상한=무제한).
D_RELIABLE_MIN = 0.24
D_RELIABLE_MAX = 10.0
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
# 거리 유효성 안전장치: d가 실제 이동(odom)보다 크게 왔다갔다 하면 '유효 거리 아님' → 도킹 중단.
D_UNSTABLE_SPREAD = 0.04         # 창 내 d(max-min)가 odom 이동보다 이만큼 더 크면 불안정 [m]
D_UNSTABLE_WIN = 8               # 판정 프레임 창(검출 프레임 기준)
D_UNSTABLE_HOLD = 0.7            # 불안정이 이만큼 지속되면 중단 [s]
CL_PLAN_BACKUP_MAX = 0.15         # PLAN서 근접이면 후진 한계 [m]
# 중심선 기동 후 재정렬 검증(VERIFY): TURN2 직후 d 가 신뢰거리 아래라 근접 노이즈로 바로
#  STAGED 하면 삐뚤어진다 → 신뢰거리로 물러나 재확인, 벗어나면 재계획(수렴 반복). 놓치면 후진
#  재획득, 한도 넘으면 SEARCH(무한대기 방지). (ddago01 실주행 검증)
CL_VERIFY_D = 0.25               # 재확인 최소 거리 [m] (yaw 신뢰거리보다 살짝 위)
CL_MAX_REPLANS = 2               # (구)재계획 한도 — yaw 좁히기 정책선 미사용(1회 시도 후 중단)
# yaw 좁히기 정책(사용자 지시): PLAN 안정 yaw 가
#  ① |yaw| ≤ YAW_NARROW_OK  → 이미 신뢰밴드 → 기동 없이 진행(ALIGN)
#  ② YAW_NARROW_OK<|yaw|≤YAW_NARROW_MAX → 1회 turn-drive-turn(횡이동+재정면) → VERIFY
#  ③ |yaw| > YAW_NARROW_MAX → 즉시 ABORT. VERIFY: 신뢰밴드면 진행, 아니면 중단(재계획 없음).
YAW_NARROW_OK = math.radians(8.0)     # 진행 허용 yaw(신뢰밴드) [rad]
YAW_NARROW_MAX = math.radians(35.0)   # 이 이상이면 시도 없이 ABORT [rad]
# PLAN 안정 확정: 튄 프레임(특히 yaw ±플립) 1장에 확정하지 않도록 최근 N프레임 yaw 가
#  좁게 모일 때만 median 값으로 확정. yaw 튐 지속 시 CL_PLAN_TIMEOUT→ALIGN 폴백.
CL_PLAN_STABLE_N = 5             # 확정에 필요한 연속 안정 프레임 수
CL_PLAN_YAW_SPREAD = math.radians(10.0)   # 이 안에 yaw 들이 모여야 안정 [rad]
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
    'D_RELIABLE_MIN', 'D_RELIABLE_MAX',
    'D_UNSTABLE_SPREAD', 'D_UNSTABLE_WIN', 'D_UNSTABLE_HOLD',
    'YAW_NARROW_OK', 'YAW_NARROW_MAX',
    'READ_HOLD_SEC', 'READ_TIMEOUT', 'READ_COOLDOWN',
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


def _median(xs):
    xs = sorted(xs)
    n = len(xs)
    if n == 0:
        return 0.0
    return xs[n // 2] if n % 2 else 0.5 * (xs[n // 2 - 1] + xs[n // 2])


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
    dist = math.hypot(gx, gy)
    # DRIVE 방향 자동선택: 전진(G 향해) vs 후진(뒤를 G로) 중 회전량 작은 쪽.
    #  G가 뒤(근접 d<CL_TARGET_D)면 전진안은 th1≈±180°(왕복)라, 후진안이 회전 0 → 헛턴 제거.
    #  dist 부호로 방향 전달: dist>0 전진, dist<0 후진(마커는 계속 정면 → 검출 유지).
    th1_f = math.atan2(gy, gx)                 # 전진: G 향해 회전
    th2_f = _ang_norm(face_dir - th1_f)
    th1_b = _ang_norm(th1_f + math.pi)         # 후진: 뒤를 G 로
    th2_b = _ang_norm(face_dir - th1_b)
    if abs(th1_b) + abs(th2_b) < abs(th1_f) + abs(th2_f):
        th1, th2, drive = th1_b, th2_b, -1.0
    else:
        th1, th2, drive = th1_f, th2_f, 1.0
    return d, bearing, yaw, (th1, drive * dist, th2), gx, gy


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
        self.read_start = None        # READ(H검출 후 정지하고 로마 숫자 읽기) 시작시각
        self.read_cooldown_until = 0.0  # 거부한 옆 스테이션 재READ 방지 종료시각(monotonic)
        self.dbuf = []                # 거리 유효성 판정용 최근 (d, ox, oy)
        self.unstable_since = None    # d 불안정 시작 시각(유효거리 아님→중단 판정)
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
        self.cl_pbuf = []             # PLAN 안정 확정용 최근 (yaw, bearing, th1, dist, th2)
        self.cl_plan = None           # (th1_deg, dist_cm, th2_deg) 로깅용
        self.cl_replans = 0           # VERIFY 재계획 횟수(한도 CL_MAX_REPLANS)
        self.cl_verify_since = None   # VERIFY 재획득 시작시각(무한대기 방지)
        self.cl_verify_xy0 = None
        self.post_advance_m = 0.0     # >0 이면 REVERSE 완료 후 HOLD→전진(반복 테스트용) → DONE
        self.hold_start = None        # HOLD(도킹완료 정지) 시작시각
        self.adv_xy0 = None
        self.adv_target = 0.0         # ADVANCE 목표 이동거리 = rev_dist + margin(창 안 착지, HOLD서 계산)
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

    def _enter_after_search(self):
        """SEARCH/READ 완료(목표 스테이션 확정) → 중심선기동 or 정렬 진입."""
        self.state = "CENTERLINE" if self.use_centerline else "ALIGN"
        self.cl_phase = "PLAN"
        self.cl_plan_since = None
        self.cl_plan_xy0 = None
        self.cl_replans = 0
        self.cl_verify_since = None
        self.lost_since = None
        self.search_start = None

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
               plan=None, odom_xy=None, n_plan=None, read_enable=False, station_ok=True):
        now = time.monotonic()
        if n_plan is None:
            n_plan = N_PLAN

        if self.state in ("DONE", "ABORT"):
            return 0.0, 0.0

        # 신뢰거리서 yaw 가 좋았는지 한 번 기록(근접 FACE 완화 근거)
        if found and d > YAW_RELIABLE_D and abs(yaw) < YAW_TOL:
            self.yaw_verified = True

        # ── 거리 유효성 안전장치 ──
        #  d가 로봇 실제 이동(odom 위치)보다 크게 왔다갔다 하면(비단조 요동) = '유효 거리 아님'
        #  → 나쁜 접근으로 진행 말고 ★도킹 중단★(안전).
        #  ★적용 = ALIGN·STAGED 만★: d를 믿고 접근/정차하는 곳. 회전·개루프 기동(SEARCH 회전·
        #  CENTERLINE turn-drive-turn)은 뷰변화로 d가 정상적으로 뜀 → 제외(오발동). PLAN/VERIFY
        #  는 자체 median 안정화 담당.
        if found and self.state in ("ALIGN", "STAGED"):
            ox, oy = odom_xy if odom_xy is not None else (0.0, 0.0)
            self.dbuf = (self.dbuf + [(d, ox, oy)])[-D_UNSTABLE_WIN:]
            if len(self.dbuf) >= D_UNSTABLE_WIN:
                ds = [b[0] for b in self.dbuf]
                spread = max(ds) - min(ds)                 # d 요동폭
                omove = 0.0                                # 창 동안 로봇 실제 이동(odom)
                if odom_xy is not None:
                    xs = [b[1] for b in self.dbuf]
                    ys = [b[2] for b in self.dbuf]
                    omove = math.hypot(max(xs) - min(xs), max(ys) - min(ys))
                if spread - omove > D_UNSTABLE_SPREAD:      # 이동으로 설명 안 되는 요동
                    self.unstable_since = self.unstable_since or now
                    if now - self.unstable_since > D_UNSTABLE_HOLD:
                        self._abort('거리 측정 불안정(Δd=%.1fcm vs odom %.1fcm, %.1fs) '
                                    '— 유효 거리 아님, 도킹 중단'
                                    % (spread * 100, omove * 100, now - self.unstable_since))
                        self.dbuf = []
                        self.unstable_since = None
                        return 0.0, 0.0
                else:
                    self.unstable_since = None
        else:
            self.dbuf = []            # 기동 아님/블라인드 → 버퍼 리셋(기동 진입 후 새로 판정)
            self.unstable_since = None

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
                # ADVANCE 거리 = 후진(rev_dist) 되돌리기 + margin → 재획득 d ≈ D_STAGE + margin.
                #  창을 안 넘게 margin 상한 클램프(≤ D_RELIABLE_MAX−D_STAGE). rev_dist 무관하게 창 안 착지
                #  → 다음 회차가 신뢰창 밖(데이터 없음)에서 시작해 복구 못 하는 문제 방지.
                margin = min(self.post_advance_m, max(0.0, D_RELIABLE_MAX - D_STAGE))
                self.adv_target = self.rev_dist + margin
            return 0.0, 0.0

        if self.state == "ADVANCE":              # 도킹 후 전진(반복용). odom 실거리, 없으면 생략
            if self.adv_xy0 is None:
                self.adv_xy0 = odom_xy
            if odom_xy is None or self.adv_xy0 is None:
                self.state = "DONE"
                return 0.0, 0.0
            trav = math.hypot(odom_xy[0] - self.adv_xy0[0], odom_xy[1] - self.adv_xy0[1])
            # 목표(adv_target=rev_dist+margin) 도달 or 전방 장애물 → 조기 정지·완료(ABORT 아님, 다음 진행)
            if trav >= self.adv_target or self.obstacle_ahead:
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
                # 계획은 [계획하한, D_RELIABLE_MAX] 서 좋은 프레임일 때만.
                #  계획하한 = max(D_RELIABLE_MIN, YAW_RELIABLE_D): 거리 신뢰창 안이라도 yaw 신뢰거리
                #  보다 가까우면(각도 부정확) 계획 금지 → 회색지대[MIN,YAW_D]는 (B)로 직진 후진해
                #  yaw 신뢰거리까지 물러난 뒤 재계획.
                cl_plan_dmin = max(D_RELIABLE_MIN, YAW_RELIABLE_D)
                if not (found and plan is not None and n >= n_plan
                        and cl_plan_dmin <= d <= D_RELIABLE_MAX):
                    self.cl_pbuf = []           # 창 이탈/미검출 → 안정버퍼 리셋(재진입 시 새로 모음)
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
                    # 너무 멂(d>MAX): 창 밖은 거리 데이터 부정확 → 접근은 backed 한계 안에서만(폭주 방지).
                    #  근본 대책은 ADVANCE 가 창을 안 넘게 하는 것(반복 도킹, adv_target 참조).
                    if d > D_RELIABLE_MAX and odom_xy is not None:
                        return V_APPROACH, _clamp(-K_BEARING * bearing, W_MAX)
                    # (B) 너무 가까움(d<MIN)엔 직진 후진만(회전 제거) — 근접 검출을 동시 회전이 깨뜨리지 않게.
                    if d < D_STAGE + CL_PLAN_BACKUP_MAX and odom_xy is not None:
                        return -V_APPROACH, 0.0
                    return 0.0, 0.0
                if odom_yaw is None:
                    self._abort('odom 없음 — 중심선 기동 불가 (bringup 미실행?)')
                    return 0.0, 0.0
                # ★안정 확정: 최근 N프레임 yaw 가 좁게 모일 때만 median 으로 확정(튄 프레임 커밋 방지).
                #  정지하고 안정 대기 → yaw ±플립 지속되면 위 CL_PLAN_TIMEOUT→ALIGN 폴백.
                self.cl_pbuf = (self.cl_pbuf
                                + [(yaw, bearing, plan[0], plan[1], plan[2])])[-CL_PLAN_STABLE_N:]
                yy = [p[0] for p in self.cl_pbuf]
                if (len(self.cl_pbuf) < CL_PLAN_STABLE_N
                        or (max(yy) - min(yy)) > CL_PLAN_YAW_SPREAD):
                    return 0.0, 0.0          # 정지하고 안정 프레임 수집 중(튐이면 확정 보류)
                myaw, mbrg = _median(yy), _median([p[1] for p in self.cl_pbuf])
                # ★yaw 좁히기 정책★
                if abs(myaw) > YAW_NARROW_MAX:        # ③ 너무 벌어짐 → 시도 없이 중단
                    self._abort('yaw 너무 벌어짐(%.1f° > %.1f°) — 1회 횡이동으로도 못 좁힘, 도킹 중단'
                                % (math.degrees(myaw), math.degrees(YAW_NARROW_MAX)))
                    self.cl_pbuf = []
                    return 0.0, 0.0
                if abs(myaw) <= YAW_NARROW_OK:        # ① 이미 신뢰밴드 → 기동 없이 진행
                    self._warn('yaw %.1f° ≤ %.1f° 신뢰밴드 — 중심선 기동 스킵 → ALIGN'
                               % (math.degrees(myaw), math.degrees(YAW_NARROW_OK)))
                    self.state = "ALIGN"
                    self.cl_done = True
                    self.lost_since = None
                    self.cl_pbuf = []
                    return 0.0, 0.0
                # ② 좁힐 수 있는 구간 → 1회 turn-drive-turn, 이후 VERIFY 판정
                self.cl_th1 = _median([p[2] for p in self.cl_pbuf])
                self.cl_dist = _median([p[3] for p in self.cl_pbuf])
                self.cl_th2 = _median([p[4] for p in self.cl_pbuf])
                self.cl_odom0 = odom_yaw
                self.cl_plan = (math.degrees(self.cl_th1), self.cl_dist * 100,
                                math.degrees(self.cl_th2))
                self.cl_phase = "TURN1"
                self.cl_pbuf = []
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
                target = abs(self.cl_dist)          # cl_dist 부호 = 방향(음수 후진)
                if odom_xy is not None and self.cl_xy0 is not None:
                    trav = math.hypot(odom_xy[0] - self.cl_xy0[0],
                                      odom_xy[1] - self.cl_xy0[1])
                    done = trav >= target
                else:
                    done = now - self.cl_drive_start >= target / V_APPROACH
                if done:
                    self.cl_phase = "TURN2"
                    return 0.0, 0.0
                # 후진 DRIVE(cl_dist<0)면 후방 블라인드 → 라이다 후방 장애물 시 정지·ALIGN 폴백.
                if self.cl_dist < 0 and self.obstacle_behind:
                    self._warn('후진 DRIVE 중 후방 장애물(라이다) — 정지·ALIGN 폴백')
                    self.state = "ALIGN"
                    return 0.0, 0.0
                w = _clamp(-K_HEADING * _ang_norm(odom_yaw - self.hold_yaw), W_MAX)
                return math.copysign(V_APPROACH, self.cl_dist), w
            if self.cl_phase == "TURN2":
                err = _ang_norm(self.cl_odom0 + self.cl_th1 + self.cl_th2 - odom_yaw)
                if abs(err) < TURN_TOL:
                    self.cl_phase = "VERIFY"   # 1회 좁힘 후 재확인 → 신뢰밴드면 진행, 아니면 중단
                    self.cl_verify_since = None
                    self.cl_pbuf = []
                    self.lost_since = None
                    return 0.0, 0.0
                return 0.0, _clamp(K_TURN * err, TURN_W)
            # 재정렬 검증(1회 좁힘 후): 안정 프레임으로 재확인 → 신뢰밴드면 진행, 아니면 중단.
            #  ★재계획 루프 없음(사용자 지시: 1회만 시도).★
            if self.cl_phase == "VERIFY":
                # 미검출/근접(yaw 불신)이면 계획을 신뢰하고 진행(후진 없이 그 자리 판정).
                if (not found) or d < YAW_RELIABLE_D:
                    self.state = "ALIGN"
                    self.cl_done = True
                    self.lost_since = None
                    self.cl_pbuf = []
                    return 0.0, 0.0
                # 튄 프레임 판정 방지: 안정 N프레임 모아 median 으로 판정.
                if self.cl_verify_since is None:
                    self.cl_verify_since = now
                    self.cl_pbuf = []
                self.cl_pbuf = (self.cl_pbuf + [(yaw, bearing)])[-CL_PLAN_STABLE_N:]
                yv = [p[0] for p in self.cl_pbuf]
                if (len(self.cl_pbuf) < CL_PLAN_STABLE_N
                        or (max(yv) - min(yv)) > CL_PLAN_YAW_SPREAD):
                    if now - self.cl_verify_since > CL_PLAN_TIMEOUT:
                        self._abort('VERIFY yaw 불안정 지속(%.1fs) — 유효 정렬 확인 불가, 도킹 중단'
                                    % (now - self.cl_verify_since))
                        self.cl_pbuf = []
                        return 0.0, 0.0
                    return 0.0, 0.0          # 정지하고 안정 프레임 수집
                myaw = _median(yv)
                mbrg = _median([p[1] for p in self.cl_pbuf])
                self.cl_pbuf = []
                if abs(mbrg) < BEARING_TOL and abs(myaw) < YAW_NARROW_OK:   # 신뢰밴드 → 진행
                    self.state = "ALIGN"
                    self.cl_done = True
                    self.lost_since = None
                    self._warn('CL VERIFY OK b=%.1f y=%.1f (1회 좁힘) → ALIGN'
                               % (math.degrees(mbrg), math.degrees(myaw)))
                    return 0.0, 0.0
                self._abort('1회 좁힘 후에도 벌어짐(b=%.1f y=%.1f, 밴드 ±%.1f°) — 도킹 중단'
                            % (math.degrees(mbrg), math.degrees(myaw),
                               math.degrees(YAW_NARROW_OK)))
                return 0.0, 0.0
            return 0.0, 0.0

        # ── 탐색 ──
        if self.state == "SEARCH":
            # read_enable=True(task_point_id '1'~'3')면 H 검출 시 정지하고 로마 숫자를 읽는다(READ).
            #  ★경사 과대(|yaw|>YAW_NARROW_MAX)면 락 후보 아님 — ID 검출/READ 없이 계속 회전.
            #  (경사 크면 로마 카운트·포즈 부정확 → 언더카운트 오매치 위험. 회전으로 다른 각/스테이션 탐색.)
            lockable = found and read_enable and abs(yaw) <= YAW_NARROW_MAX
            if lockable:
                if now < self.read_cooldown_until:
                    return 0.0, SEARCH_W          # 방금 거부한 옆 스테이션 — 회전해 지나침
                self.state = "READ"               # H 검출 + 경사 OK → 정지하고 로마 숫자 읽기
                self.read_start = now
                self.search_start = None
                return 0.0, 0.0
            if found and not read_enable:
                self._enter_after_search()        # 로마 인식 미사용 → 즉시 진행(기존 동작)
            else:
                # 미검출 or 경사 과대 → 회전 계속(+타임아웃 시 중단)
                if self.search_start is None:
                    self.search_start = now
                if now - self.search_start > SEARCH_TIMEOUT:
                    self._abort('탐색 실패 — %.0f초(%.1f바퀴) 회전(마커 미검출 또는 경사 과대>%.0f°)'
                                % (SEARCH_TIMEOUT, SEARCH_REVS, math.degrees(YAW_NARROW_MAX)),
                                code=1)
                    return 0.0, 0.0
                return 0.0, SEARCH_W

        # ── H 검출 후 정지하고 로마 숫자(스테이션 ID) 읽기 ──
        if self.state == "READ":
            if not found:                         # 읽는 중 H 놓침 → 재탐색
                self.state = "SEARCH"
                self.read_start = None
                self.search_start = None
                return 0.0, 0.0
            waited = now - (self.read_start or now)
            if station_ok and waited >= READ_HOLD_SEC:
                self._enter_after_search()        # 최소 HOLD 정지 + 목표ID 일치 → 진행
                self.read_start = None
                return 0.0, 0.0
            if waited > READ_TIMEOUT:             # 합의 실패(옆 스테이션 등) → 지나쳐 재탐색
                self.state = "SEARCH"
                self.read_start = None
                self.search_start = now
                self.read_cooldown_until = now + READ_COOLDOWN
                return 0.0, SEARCH_W
            return 0.0, 0.0                       # 계속 정지하고 읽는다

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
            # 목표(D_STAGE=신뢰창 하한+5mm) 근처면 감속 → 신뢰창(H 검출 한계) 아래로 오버슛 방지.
            v = V_APPROACH * max(V_STAGE_MIN_FRAC, min(1.0, (d - D_STAGE) / V_DECEL_ZONE))
            return v, w_align

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
