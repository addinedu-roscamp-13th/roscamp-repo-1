#!/usr/bin/env python3
"""반사테이프 후진 도킹 FSM — 순수 로직 (ROS 의존 없음).

dock_controller.py 의 상태기계를 ROS(노드·퍼블리셔·타이머·shutdown)에서 떼어내,
'관측(마커 pose + odom)을 먹이면 (v, w) 명령을 돌려주는' 순수 클래스로 추출했다.
floor_dock 의 floor_fsm.py 와 같은 패턴이다:
  * 이 모듈은 로봇/ROS 없이 단위 테스트된다(test_reflective_dock.py).
  * reflective_dock_server.py(액션 서버)와 dock_controller.py(디버그 노드)가
    둘 다 이 하나의 FSM 을 구동한다 → 로직 출처가 하나(드리프트 방지).

상태 흐름 (feedback.phase 값과 1:1):
  SNAP     멈춘 채 마커 pose 를 평균 → 법선(odom)을 스냅, 수선의 발 P 계산
  TURN1    제자리 회전으로 P 를 바라봄
  DRIVE    P 까지 직진(옆으로 벗어난 만큼)
  TURN2    뒤축이 마커를 향하도록(법선) 회전            ── 여기까지 사전정렬
  APPROACH 후진하며 β(중앙 보기)·e(법선 이탈) 동시 교정
  CREEP    마지막 블라인드 후진(odom 실이동거리)        ── 여기부터 후진
  → 종료(self.done). success=True 면 성공.

⚠️ 로그는 이 모듈에서 하지 않는다(순수 로직). 호출부(서버/노드)가 state 전이와
   self.note 를 보고 로깅한다.
"""
import math

# ── 라이다 오프셋 (URDF 실측, detector_node.to_odom 와 동일) ──
# 라이다가 회전중심 뒤 1.7cm 에 있고 0°축이 섀시 뒤(180°)를 본다.
LIDAR_X = -0.017
LIDAR_Y = 0.0
LIDAR_YAW_OFFSET = math.pi

# ── 사전정렬(SNAP~TURN2) 튜닝 ──
SNAP_FRAMES = 15    # 법선 스냅에 평균낼 프레임 수
KW = 1.2            # 회전 비례게인
KV = 0.4            # 직진 비례게인
TURN_TOL_DEG = 3.0  # 회전 완료 허용오차
DRIVE_TOL_M = 0.01  # 직진 완료 허용오차
MIN_MOVE_M = 0.02   # 이보다 적게 벗어났으면 DRIVE 생략(이미 법선 위)
MAX_MOVE_M = 0.30   # 이보다 크게 나오면 스냅샷 이상 → 안전 중단
WMAX = 0.4          # 회전 상한(사전정렬)
WMIN = 0.15         # 데드밴드 극복 최소 회전

# ── 후진(APPROACH~CREEP) 튜닝 ──
VMAX = 0.05
VMIN = 0.015
KDIST = 0.4         # 거리 오차 → 후진속도 게인
KBETA = 0.8         # β(중앙 보기) 게인
KE = 0.7            # e(법선 끌어당김) 게인
SIGN_E = +1.0       # e 교정 방향(실측 +1)
MAX_BETA_DEG = 60.0 # |β| 과대면 오검출로 무시
WCAP = 0.3          # 회전 상한(후진)
SWITCH_M = 0.18     # 이 거리 이하면 크립 전환
CREEP_V = 0.015
CREEP_MAX = 0.09
DMAX_MARGIN = 0.03  # 거리가 시작값보다 이만큼 늘면 방향 이상 → 중단

# ── 공통 ──
WATCHDOG_SEC = 0.5  # 호출부가 마커 pose 신선도 판정에 쓰는 기준(참고용 기본값)
MAX_SEC = 60.0      # 도킹 전체 제한시간

# ── Result.result_code (ReflectiveDock.action 주석과 일치) ──
RC_OK = 0
RC_MARKER_NOT_FOUND = 1
RC_TOLERANCE = 2
RC_CANCELLED = 3
RC_ALIGN_FAILED = 4

# configure() 로 런타임(노드 파라미터/goal) 덮어쓰기 허용하는 상수들.
_TUNABLE = (
    'SNAP_FRAMES', 'KW', 'KV', 'TURN_TOL_DEG', 'DRIVE_TOL_M', 'MIN_MOVE_M',
    'MAX_MOVE_M', 'WMAX', 'WMIN', 'VMAX', 'VMIN', 'KDIST', 'KBETA', 'KE',
    'SIGN_E', 'MAX_BETA_DEG', 'WCAP', 'SWITCH_M', 'CREEP_V', 'CREEP_MAX',
    'DMAX_MARGIN', 'MAX_SEC',
)


def configure(**kwargs):
    """모듈 튜닝 상수를 덮어쓴다(floor_fsm.configure 와 같은 방식).

    액션 서버가 노드 파라미터로 게인을 조정할 때 쓴다. 단일 인스턴스 도킹이라
    모듈 전역을 바꿔도 안전하다(floor_fsm 과 동일 전제).
    """
    g = globals()
    for k, v in kwargs.items():
        if k not in _TUNABLE:
            raise KeyError('알 수 없는 튜닝 상수: %s' % k)
        g[k] = v


def norm_ang(a):
    """각도를 -pi~pi 로 정규화."""
    return math.atan2(math.sin(a), math.cos(a))


class ReflectiveDockFsm:
    """반사 도킹 상태기계. step(pose, odom, now) 로 한 틱씩 굴린다."""

    def __init__(self, rear_offset_m=0.10, stop_gap_m=0.02):
        # 로봇 물리값(로봇별 config yaml). 정지 목표 라이다거리 = rear + gap.
        self.rear_offset_m = float(rear_offset_m)
        self.stop_gap_m = float(stop_gap_m)
        self.target_m = self.rear_offset_m + self.stop_gap_m
        self.d_emergency = self.rear_offset_m + 0.005  # 비상정지 거리

        self.state = 'SNAP'
        # 사전정렬용
        self.snap = []           # (ox, oy, cos n, sin n) 프레임들
        self.P = None            # 법선 위 수선의 발(odom)
        self.theta_final = None  # 최종 헤딩(법선)
        self.heading_to_P = None
        self.snap_move = None     # 법선까지 이동거리(로깅용)
        # 후진용
        self.d0 = None
        self.last_good_t = None
        self.creep_origin = None
        self.creep_dist = 0.0

        # 종료/결과
        self.done = False
        self.success = False
        self.result_code = None
        self.note = ''
        self.final_gap = 0.0        # 뒤끝~마커 [m]
        self.final_lateral = 0.0    # e (cross-track) [m]
        self.final_yaw_error = 0.0  # tilt [rad]

        # feedback/진단용
        self.last_d = None          # 라이다~마커 거리 [m]
        self.marker_detected = False
        self.start_t = None
        self._seen_marker = False   # 마커를 한 번이라도 봤나(타임아웃 코드 판정용)

    # ------------------------------------------------------------------ #
    def _finish(self, reason, code, success=False):
        self.done = True
        self.success = success
        self.result_code = code
        self.note = reason
        return (0.0, 0.0)

    def to_odom(self, x_l, y_l, odom):
        """라이다 프레임 점 → odom 좌표. 반환 (ox, oy, a). a=라이다헤딩(odom)."""
        X, Y, th = odom
        lox = X + LIDAR_X * math.cos(th) - LIDAR_Y * math.sin(th)
        loy = Y + LIDAR_X * math.sin(th) + LIDAR_Y * math.cos(th)
        a = th + LIDAR_YAW_OFFSET
        ox = lox + x_l * math.cos(a) - y_l * math.sin(a)
        oy = loy + x_l * math.sin(a) + y_l * math.cos(a)
        return ox, oy, a

    # ------------------------------------------------------------------ #
    def step(self, pose, odom, now):
        """관측 한 틱을 먹여 진행하고 (v, w) 를 반환한다.

        pose : (x, y, yaw) 마커(라이다프레임). 신선하지 않으면 None.
        odom : (X, Y, yaw) 로봇 odom pose. 아직 없으면 None.
        now  : 단조시계 초(time.monotonic()).
        """
        if self.done:
            return (0.0, 0.0)
        if self.start_t is None:
            self.start_t = now
        self.marker_detected = pose is not None

        if now - self.start_t > MAX_SEC:
            code = RC_ALIGN_FAILED if self._seen_marker else RC_MARKER_NOT_FOUND
            return self._finish('전체 제한시간 초과', code)
        if odom is None:
            return (0.0, 0.0)  # odom 대기(정지)

        s = self.state
        if s == 'SNAP':
            return self._snap(pose, odom)
        if s == 'TURN1':
            return self._turn(self.heading_to_P, 'DRIVE', odom)
        if s == 'DRIVE':
            return self._drive(odom)
        if s == 'TURN2':
            return self._turn(self.theta_final, 'APPROACH', odom)  # ★ 후진으로 이어짐
        if s == 'APPROACH':
            return self._approach(pose, odom, now)
        if s == 'CREEP':
            return self._creep(odom)
        return (0.0, 0.0)

    # ---- SNAP~TURN2 (사전정렬) ---------------------------------------- #
    def _snap(self, pose, odom):
        if pose is None:
            return (0.0, 0.0)  # 스냅샷 동안 정지, 마커 대기
        self._seen_marker = True
        x, y, yaw_l = pose
        ox, oy, a = self.to_odom(x, y, odom)
        n_odom = yaw_l + a  # 라이다프레임 법선 → odom 방향
        self.snap.append((ox, oy, math.cos(n_odom), math.sin(n_odom)))
        if len(self.snap) < SNAP_FRAMES:
            return (0.0, 0.0)
        k = len(self.snap)
        mx = sum(s[0] for s in self.snap) / k
        my = sum(s[1] for s in self.snap) / k
        n = math.atan2(sum(s[3] for s in self.snap), sum(s[2] for s in self.snap))
        return self._compute_goal(mx, my, n, odom)

    def _compute_goal(self, mx, my, n, odom):
        """마커 꼭짓점 M·법선 n → 수선의 발 P·최종 헤딩."""
        rx, ry, _ = odom
        ux, uy = math.cos(n), math.sin(n)
        proj = (rx - mx) * ux + (ry - my) * uy
        px, py = mx + proj * ux, my + proj * uy
        move = math.hypot(px - rx, py - ry)
        self.P = (px, py)
        self.theta_final = n
        self.heading_to_P = math.atan2(py - ry, px - rx)
        self.snap_move = move
        if move > MAX_MOVE_M:
            return self._finish(
                '스냅샷 이동거리 %.0fcm 과대 — 중단' % (move * 100), RC_ALIGN_FAILED)
        self.state = 'TURN2' if move < MIN_MOVE_M else 'TURN1'
        return (0.0, 0.0)

    def _turn(self, target_th, next_state, odom):
        _, _, rth = odom
        err = norm_ang(target_th - rth)
        if abs(err) < math.radians(TURN_TOL_DEG):
            self.state = next_state
            return (0.0, 0.0)
        wz = max(-WMAX, min(WMAX, KW * err))
        if abs(wz) < WMIN:
            wz = math.copysign(WMIN, err)
        return (0.0, wz)

    def _drive(self, odom):
        rx, ry, rth = odom
        rem = math.hypot(self.P[0] - rx, self.P[1] - ry)
        head = math.atan2(self.P[1] - ry, self.P[0] - rx)
        err = norm_ang(head - rth)
        # 남은 거리가 짧거나, 목표가 뒤로 넘어가(err>90°) 지나쳤으면 종료.
        if rem < DRIVE_TOL_M or abs(err) > math.radians(90.0):
            self.state = 'TURN2'
            return (0.0, 0.0)
        v = max(VMIN, min(VMAX, KV * rem))
        wz = max(-WMAX, min(WMAX, KW * err))
        return (v, wz)

    # ---- APPROACH~CREEP (후진) --------------------------------------- #
    def _approach(self, pose, odom, now):
        if pose is None:
            # 한 번이라도 좋은 값을 본 뒤에만 상실 판정(TURN2 직후 재획득 유예).
            if self.last_good_t is not None and (now - self.last_good_t) > 0.7:
                return self._finish('마커 상실 — 정지', RC_MARKER_NOT_FOUND)
            return (0.0, 0.0)

        x, y, yaw = pose
        d = math.hypot(x, y)
        beta = math.atan2(y, x)
        e = -x * math.sin(yaw) + y * math.cos(yaw)  # 법선축 이탈(cross-track)
        tilt = abs(((math.degrees(yaw) - 180.0 + 180.0) % 360.0) - 180.0)

        if abs(beta) > math.radians(MAX_BETA_DEG):  # 스퓨리어스 차단
            if self.last_good_t is not None and (now - self.last_good_t) > 1.0:
                return self._finish('마커 이상(β 과대) — 정지', RC_MARKER_NOT_FOUND)
            return (0.0, 0.0)

        self.last_good_t = now
        self._seen_marker = True
        self.last_d = d
        self.final_lateral = e
        self.final_yaw_error = math.radians(tilt)

        if self.d0 is None:
            self.d0 = d
        if d < self.d_emergency:
            self.final_gap = max(0.0, d - self.rear_offset_m)
            return self._finish('비상정지 d=%.1fcm' % (d * 100), RC_TOLERANCE)
        if d > self.d0 + DMAX_MARGIN:
            return self._finish('거리 증가 d=%.1fcm — 방향 이상' % (d * 100), RC_ALIGN_FAILED)
        if d <= SWITCH_M:
            self.creep_origin = (odom[0], odom[1])
            self.creep_dist = max(0.0, min(CREEP_MAX, d - self.target_m))
            self.state = 'CREEP'
            return (0.0, 0.0)

        v = max(VMIN, min(VMAX, KDIST * (d - self.target_m)))
        wz = max(-WCAP, min(WCAP, KBETA * beta + SIGN_E * KE * e))
        return (-v, wz)

    def _creep(self, odom):
        if self.creep_origin is None:
            return (0.0, 0.0)
        traveled = math.hypot(
            odom[0] - self.creep_origin[0], odom[1] - self.creep_origin[1])
        if traveled >= self.creep_dist:
            self.final_gap = self.stop_gap_m
            return self._finish(
                '도킹 완료(크립 %.1fcm) → 뒤끝 여유≈%.0fcm'
                % (traveled * 100, self.stop_gap_m * 100), RC_OK, success=True)
        return (-CREEP_V, 0.0)
