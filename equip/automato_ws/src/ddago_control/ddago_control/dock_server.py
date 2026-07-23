#!/usr/bin/env python3
"""RP-102  E4-6 / E2 22-1: DdaGo(주행 로봇) Dock Action 서버 — 충전소 정밀 후진 도킹.

ACS 가 충전소 진입 노드까지 Navigate 로 데려다 놓은 뒤 Dock goal 을 하달하면(E4 6),
로봇은 **앞面 카메라로 ChArUco 보드를 보면서 뒷面을 스테이션에 붙인다**(후진 도킹).

  * Dock 액션의 **서버**        (DCS → DdaGo, /ddago/dock)
  * 정면 picamera(CSI) **직접 사용** (picamera2 — 측면 순찰 웹캠과 별개 장치다)
  * odom 토픽 **구독자**        (개루프 기동의 회전·거리 기준)
  * cmd_vel **발행자**          (주행 명령)

액션 이름을 절대이름 `/ddago/dock` 으로 두는 이유는 navigate_server 와 같다 —
로봇 쪽 이름에는 robot_id 네임스페이스를 붙이지 않는다.

기동 순서 (Dock.action 의 phase 값과 1:1):
  SEARCHING   제자리 회전하며 보드 탐색
  CENTERING   보드 중심선 위로 이동 (turn-drive-turn, 개루프 odom)
  APPROACHING 스테이징 거리까지 전진 + 정면 미세정렬
  STAGED      스테이징 완료(정지) — 곧바로 다음 단계로
  ROTATING    제자리 180도 회전 (뒷面이 스테이션을 향하게)
  REVERSING   후진 접붙임 (odom 실이동거리 + 직진성 유지)

**CENTERING 이 핵심이다.** 카메라를 보드 중앙에 두고 전진하는 단순 추종(bearing 추종)만
쓰면, 중심선에서 벗어나 배치됐을 때 호를 그리며 접근해 **비스듬히 도착**한다. 그 잔류
이탈은 제자리 회전으로 못 고친다(비홀로노믹 — 중심선 이탈은 옆으로 이동해야 해소된다).
그래서 좋은 검출 한 장으로 중심선 위 스테이징점 G 까지의 경로(회전-직진-회전)를 미리
계산해 **개루프 odom 으로** 실행한 뒤, 마지막에 비전으로 미세정렬한다. 기동 중 보드가
화각을 벗어나도 무방하다.

중심선 이탈 정도는 **sigma = bearing - yaw** 로 알 수 있다(실측 검증: 이탈거리 ~= d*sin(sigma)).
제어에는 쓰지 않고 진단/결과보고(final_lateral_m)에만 쓴다.

**DRIVE·REVERSE 는 시간이 아니라 odom 실이동거리로 끝낸다.** 시간 기반이면 실제 속도가
명령값보다 느린 만큼(실측 ~14%) 짧게 가서 중심선에 못 미친다.

⚠️ 안전: bringup 에 cmd_vel 워치독이 없다. 이 노드는 종료·취소·실패 어느 경로로 빠져도
반드시 0 속도를 반복 발행한다(_stop). 처음 현장 투입 시에는 dry_run:=true 로 명령만
확인한 뒤 실주행할 것.

파라미터:
  robot_id            (str)   로그 표기용 로봇 식별자           기본 'dg_01'
  camera_calib_file   (str)   카메라 내부파라미터 npz(mtx,dist)  기본 charuco_dock_ws 것
  camera_width        (int)   캡처 폭  — 캘리브와 같아야 한다     기본 1280
  camera_height       (int)   캡처 높이 — 캘리브와 같아야 한다     기본 720
  odom_topic          (str)   odom 토픽(상대)                   기본 'odom'
  cmd_vel_topic       (str)   주행 명령 토픽(상대)              기본 'cmd_vel'
  rotate_180          (bool)  카메라가 180도 뒤집혀 장착됨       기본 True
  dry_run             (bool)  참이면 cmd_vel 을 발행하지 않음    기본 False
  staging_distance    (float) 스테이징 거리(카메라-보드중심)[m]  기본 0.24
  reverse_distance    (float) 후진 거리 [m]                     기본 0.15
  control_hz          (float) 제어 주기 [Hz]                    기본 12.0
  (그 외 속도/게인/허용오차는 아래 상수 기본값을 파라미터로 덮어쓸 수 있다)
"""
import fcntl
import math
import os
import sys
import threading
import time

from automato_interfaces.action import Dock
import cv2
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Int64

# 같은 액션 이름으로 서버가 둘 뜨는 것을 막는 락 파일. 환경변수로 바꿀 수 있다.
LOCK_PATH = os.environ.get('DDAGO_DOCK_LOCK', '/tmp/ddago_dock_server.lock')


def acquire_single_instance(path=LOCK_PATH):
    """단일 인스턴스 락. (fd, None) 성공 / (None, 점유PID) 실패.

    ROS2 는 동일 액션 이름(/ddago/dock)의 서버가 둘 떠도 막지 않는다. 그러면 goal 이
    어느 서버로 갈지 알 수 없고, 실제로 '정리 안 된 옛 노드가 goal 을 받아 로봇을
    움직이는' 사고가 났다. 프로세스 내부 락(_busy)은 다른 프로세스를 막지 못하므로
    파일 락으로 프로세스 간 배타를 건다.

    flock 은 프로세스가 어떻게 죽든(kill -9 포함) 커널이 풀어주므로 stale 락이
    남지 않는다. PID 파일을 직접 비교하는 방식보다 안전하다.
    """
    fd = os.open(path, os.O_RDWR | os.O_CREAT, 0o644)
    try:
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except OSError:
        try:
            holder = os.read(fd, 32).decode(errors='replace').strip() or '?'
        except OSError:
            holder = '?'
        os.close(fd)
        return None, holder
    os.ftruncate(fd, 0)
    os.write(fd, str(os.getpid()).encode())
    return fd, None


# ---- Result.result_code (Dock.action 주석과 일치) ----
RC_OK = 0
RC_MARKER_NOT_FOUND = 1
RC_TOLERANCE = 2
RC_CANCELLED = 3
RC_ALIGN_FAILED = 4

# ---- 현장 튜닝값 (ddago03 실주행 검증) ----
DEF = {
    'staging_distance': 0.24,   # 스테이징 거리(카메라-보드중심) [m]
    'reverse_distance': 0.15,   # 후진 거리 [m] — odom 실거리라 명령값=실제거리(1:1)
    'v_approach': 0.05,         # 전진 속도 [m/s]
    'v_reverse': 0.05,          # 후진 속도 [m/s]
    'search_w': 0.2,            # 탐색 회전 [rad/s] — 빠르면 마커를 지나친다
    'search_revs': 1.1,         # 이 바퀴수 넘게 못 찾으면 실패
    'turn_w': 0.25,             # 회전 상한 [rad/s] — 느릴수록 오버슈트↓
    'turn_tol_deg': 0.8,        # 회전 완료 허용오차 — 스큐에 직결되므로 타이트하게
    'k_turn': 1.2,
    'k_bearing': 1.0,
    'k_heading': 1.5,           # 직진성 유지(odom yaw)
    'w_max': 0.5,
    'bearing_tol_deg': 3.0,     # 정면 허용오차
    'yaw_tol_deg': 5.0,         # 수직(중심선) 허용오차
    'face_timeout': 6.0,        # 이 시간 내 정렬 못 하면 실패(삐뚤 도킹 방지)
    'lost_timeout': 1.0,        # 놓친 뒤 재탐색까지
    'plan_timeout': 8.0,        # 계획용 좋은 프레임 대기 상한
    'n_plan': 10,               # 계획 신뢰 최소 코너수(법선 정확도)
    'min_corners': 4,           # 검출 인정 최소 코너수
    'control_hz': 12.0,         # 제어 주기 — Pi4 CPU 과부하 방지
}


def _ang_norm(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def _clamp(x, lim):
    return max(-lim, min(lim, x))


def _yaw_from_quat(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y),
                      1 - 2 * (q.y * q.y + q.z * q.z))


class CharucoTracker:
    """Goal 이 지정한 ChArUco 보드를 검출해 도킹용 상대자세를 낸다.

    보드 규격(칸수/칸크기/마커크기/딕셔너리/시작ID)은 ACS 가 DB 에서 조회해 Goal 로
    내려준다. 로봇에 하드코딩하지 않는다.
    """

    def __init__(self, goal, min_corners):
        dic_name = goal.dictionary or 'DICT_5X5_1000'
        if not hasattr(cv2.aruco, dic_name):
            raise ValueError('알 수 없는 딕셔너리: %s' % dic_name)
        dictionary = cv2.aruco.getPredefinedDictionary(
            getattr(cv2.aruco, dic_name))

        sx, sy = int(goal.squares_x), int(goal.squares_y)
        if sx < 2 or sy < 2:
            raise ValueError('보드 칸수가 올바르지 않다: %dx%d' % (sx, sy))
        square = float(goal.square_size_m)
        marker = float(goal.marker_size_m)
        if square <= 0 or marker <= 0 or marker >= square:
            raise ValueError('칸/마커 크기가 올바르지 않다: %s/%s' % (square, marker))

        start = int(goal.marker_id) if str(goal.marker_id).strip() else 0
        n_ids = (sx * sy) // 2
        self.board = cv2.aruco.CharucoBoard(
            (sx, sy), square, marker, dictionary,
            ids=np.arange(start, start + n_ids))

        dp = cv2.aruco.DetectorParameters()
        # 근접(마커가 크게 보일 때) 검출을 살리는 핵심 값. step 을 키워 윈도우 수를
        # 줄이면 근접 검출 범위는 유지하면서 CPU 를 아낀다.
        dp.adaptiveThreshWinSizeMax = 55
        dp.adaptiveThreshWinSizeStep = 16
        self.detector = cv2.aruco.CharucoDetector(
            self.board, cv2.aruco.CharucoParameters(), dp)

        self.min_corners = int(min_corners)
        # 도킹 목표점 = 보드 중심(+ 마커 기준 좌우 오프셋). 보드 좌하단 원점 기준.
        self.target_offset = np.array(
            [sx * square / 2.0 + float(goal.dock_offset_y),
             sy * square / 2.0, 0.0])

    def detect(self, gray, mtx, dist):
        """(d, bearing, yaw, n, rvec, tvec) 또는 None.

        d       : 목표점까지 수평거리 [m]
        bearing : 목표점이 광축에서 좌우로 벗어난 각 [rad] (+우)
        yaw     : 보드 법선과 광축이 이루는 각 [rad]
        """
        try:
            ch_c, ch_ids, _, _ = self.detector.detectBoard(gray)
        except cv2.error:
            return None
        if ch_ids is None or len(ch_ids) < self.min_corners:
            return None
        try:
            obj_pts, img_pts = self.board.matchImagePoints(ch_c, ch_ids)
            # ChArUco 코너는 한 평면 위라 IPPE 로 4점부터 풀린다(기본 DLT 는 6점 필요).
            ok, rvec, tvec = cv2.solvePnP(obj_pts, img_pts, mtx, dist,
                                          flags=cv2.SOLVEPNP_IPPE)
        except cv2.error:
            return None
        if not ok or not (np.all(np.isfinite(rvec)) and np.all(np.isfinite(tvec))):
            return None   # nan 포즈 방어

        R, _ = cv2.Rodrigues(rvec)
        target = tvec.ravel() + R @ self.target_offset
        d = float(np.linalg.norm(target[[0, 2]]))
        bearing = math.atan2(target[0], target[2])
        z_axis = R[:, 2]
        yaw = math.atan2(z_axis[0], z_axis[2])
        return d, bearing, yaw, len(ch_ids), rvec, tvec

    def centerline_plan(self, rvec, tvec, staging):
        """중심선 스테이징점 G 까지의 turn-drive-turn 계획 (로봇 프레임).

        반환 (th1, dist, th2):
          th1  : G 를 향해 제자리 회전할 각 [rad] (좌+ = odom 증가방향)
          dist : G 까지 직진 거리 [m]
          th2  : G 에서 보드 정면을 향하도록 추가 회전할 각 [rad]

        G 는 보드 중심에서 법선을 따라 staging 만큼 나온 점이라 **항상 중심선 위**다.
        로봇이 옆에서 비스듬히 보고 있어도 G 의 위치는 보드 자세만으로 정해진다.
        """
        R, _ = cv2.Rodrigues(rvec)
        C = tvec.ravel() + R @ self.target_offset
        Cr, Cf = float(C[0]), float(C[2])
        nvec = R[:, 2]
        nr, nf = float(nvec[0]), float(nvec[2])
        if nr * Cr + nf * Cf > 0:            # 법선이 로봇 쪽을 향하게
            nr, nf = -nr, -nf
        nn = math.hypot(nr, nf) or 1.0
        nr, nf = nr / nn, nf / nn
        # 카메라(x=우, z=전방) -> 로봇(x=전방, y=좌)
        Cx, Cy = Cf, -Cr
        Nx, Ny = nf, -nr
        gx = Cx + staging * Nx
        gy = Cy + staging * Ny
        th1 = math.atan2(gy, gx)
        dist = math.hypot(gx, gy)
        face = math.atan2(-Ny, -Nx)          # 보드로 들어가는 방향 = 최종 정면 heading
        return th1, dist, _ang_norm(face - th1)


class DockFsm:
    """도킹 상태머신. ROS 와 분리되어 있어 단위 테스트가 가능하다.

    update(...) -> (v, w). phase 는 Dock.action 의 feedback.phase 값과 같다.
    끝나면 self.done 이 True 가 되고 self.result_code 가 채워진다.
    """

    def __init__(self, cfg, staging, reverse_dist, log=None):
        self.cfg = cfg
        self.staging = staging
        self.reverse_dist = reverse_dist
        self._log = log or (lambda _m: None)

        self.phase = 'SEARCHING'
        self.done = False
        self.result_code = RC_OK
        self.message = ''

        self._search_start = None
        self._lost_since = None
        self._face_start = None
        # CENTERING 내부 단계: PLAN -> TURN1 -> DRIVE -> TURN2
        self._cl = 'PLAN'
        self._cl_since = None
        self._cl_yaw0 = None
        self._cl_xy0 = None
        self._th1 = self._dist = self._th2 = 0.0
        self._turn_target = None
        self._hold_yaw = None
        self._rev_xy0 = None
        # 마지막 유효 관측 (결과 보고용)
        self.last_d = None
        self.last_bearing = None
        self.last_yaw = None

    # -- 종료 헬퍼 -------------------------------------------------------
    def _finish(self, code, msg):
        self.done = True
        self.result_code = code
        self.message = msg
        self._log(msg)
        return 0.0, 0.0

    def update(self, now, found, obs, plan, odom_yaw, odom_xy):
        """obs = (d, bearing, yaw, n) 또는 None, plan = (th1, dist, th2) 또는 None."""
        c = self.cfg
        if self.done:
            return 0.0, 0.0

        # ---------- 비전 무관 개루프 구간 ----------
        if self.phase == 'ROTATING':
            if odom_yaw is None:
                return self._finish(RC_ALIGN_FAILED, 'odom 없음 — 회전 불가')
            err = _ang_norm(self._turn_target - odom_yaw)
            if abs(err) < c['turn_tol']:
                self.phase = 'REVERSING'
                self._hold_yaw = odom_yaw
                self._rev_xy0 = odom_xy
                return -c['v_reverse'], 0.0
            return 0.0, _clamp(c['k_turn'] * err, c['turn_w'])

        if self.phase == 'REVERSING':
            if odom_xy is None or self._rev_xy0 is None:
                return self._finish(RC_ALIGN_FAILED, 'odom 없음 — 후진 거리 측정 불가')
            trav = math.hypot(odom_xy[0] - self._rev_xy0[0],
                              odom_xy[1] - self._rev_xy0[1])
            if trav >= self.reverse_dist:
                return self._finish(RC_OK, '도킹 완료')
            w = 0.0
            if odom_yaw is not None and self._hold_yaw is not None:
                w = _clamp(-c['k_heading'] * _ang_norm(odom_yaw - self._hold_yaw),
                           c['w_max'])
            return -c['v_reverse'], w

        if self.phase == 'STAGED':
            if odom_yaw is None:
                return self._finish(RC_ALIGN_FAILED, 'odom 없음 — 회전 불가')
            self.phase = 'ROTATING'
            self._turn_target = _ang_norm(odom_yaw + math.pi)
            return 0.0, 0.0

        if self.phase == 'CENTERING':
            return self._centering(now, found, obs, plan, odom_yaw, odom_xy)

        # ---------- 탐색 ----------
        if self.phase == 'SEARCHING':
            if found:
                self.phase = 'CENTERING'
                self._cl, self._cl_since = 'PLAN', None
                self._search_start = self._lost_since = None
                return 0.0, 0.0
            if self._search_start is None:
                self._search_start = now
            if now - self._search_start > c['search_timeout']:
                return self._finish(RC_MARKER_NOT_FOUND,
                                    '탐색 %.0f초 내 마커 미검출' % c['search_timeout'])
            return 0.0, c['search_w']

        # ---------- APPROACHING (전진 접근 + 정면 미세정렬) ----------
        if not found:
            # 스테이징 근처에서 정렬된 채로 놓쳤으면 그 자리를 스테이징으로 인정.
            if (self.last_d is not None and self.last_d < self.staging + 0.05
                    and abs(self.last_bearing) < c['bearing_tol']
                    and abs(self.last_yaw) < c['yaw_tol']):
                self.phase = 'STAGED'
                return 0.0, 0.0
            if self._lost_since is None:
                self._lost_since = now
            if now - self._lost_since > c['lost_timeout']:
                self.phase = 'SEARCHING'
                self._search_start = None
            return 0.0, 0.0

        self._lost_since = None
        d, bearing, yaw, n = obs
        self.last_d, self.last_bearing, self.last_yaw = d, bearing, yaw
        w_align = _clamp(-c['k_bearing'] * bearing, c['w_max'])
        aligned = abs(bearing) < c['bearing_tol'] and abs(yaw) < c['yaw_tol']

        if self._face_start is None:
            # 접근: 목표거리 도달 또는 검출이 바닥까지 떨어졌는데 정렬됨 -> 미세정렬로
            if d <= self.staging or (n < c['n_stage_floor'] and aligned):
                self._face_start = now
                return 0.0, w_align
            return c['v_approach'], w_align

        # 미세정렬: 제자리에서 bearing 만 다듬는다. 이 시점엔 중심선 위라(sigma~=0)
        # bearing 을 0 으로 만들면 yaw 도 함께 0 으로 간다.
        if aligned:
            self.phase = 'STAGED'
            return 0.0, 0.0
        if now - self._face_start > c['face_timeout']:
            # 삐뚤게 붙이느니 멈춘다. 대개 중심선 이탈이 남은 경우다.
            return self._finish(
                RC_ALIGN_FAILED,
                '정렬 실패 (bearing %.1f°, yaw %.1f°) — 중심선 이탈 잔류'
                % (math.degrees(bearing), math.degrees(yaw)))
        return 0.0, w_align

    # -- CENTERING (turn-drive-turn, 개루프) -----------------------------
    def _centering(self, now, found, obs, plan, odom_yaw, odom_xy):
        c = self.cfg
        if self._cl == 'PLAN':
            if self._cl_since is None:
                self._cl_since = now
            if now - self._cl_since > c['plan_timeout']:
                # 계획용 프레임을 못 얻었으면 단순 접근으로 폴백(중심선 보정 없이).
                self._log('계획 프레임 확보 실패 → 접근으로 폴백')
                self.phase = 'APPROACHING'
                return 0.0, 0.0
            if not (found and plan is not None and obs[3] >= c['n_plan']):
                return 0.0, 0.0          # 좋은 프레임 대기 (정지)
            if odom_yaw is None:
                return self._finish(RC_ALIGN_FAILED, 'odom 없음 — 중심선 기동 불가')
            self._th1, self._dist, self._th2 = plan
            self._cl_yaw0 = odom_yaw
            self._cl = 'TURN1'
            self._log('중심선 계획: 회전 %+.1f° → 직진 %.1fcm → 회전 %+.1f°'
                      % (math.degrees(self._th1), self._dist * 100,
                         math.degrees(self._th2)))
            return 0.0, 0.0

        if odom_yaw is None:
            return self._finish(RC_ALIGN_FAILED, 'odom 없음 — 중심선 기동 불가')

        if self._cl == 'TURN1':
            err = _ang_norm(self._cl_yaw0 + self._th1 - odom_yaw)
            if abs(err) < c['turn_tol']:
                self._cl = 'DRIVE'
                self._cl_xy0 = odom_xy
                self._hold_yaw = odom_yaw
                return 0.0, 0.0
            return 0.0, _clamp(c['k_turn'] * err, c['turn_w'])

        if self._cl == 'DRIVE':
            if odom_xy is None or self._cl_xy0 is None:
                return self._finish(RC_ALIGN_FAILED, 'odom 없음 — 이동거리 측정 불가')
            trav = math.hypot(odom_xy[0] - self._cl_xy0[0],
                              odom_xy[1] - self._cl_xy0[1])
            if trav >= self._dist:
                self._cl = 'TURN2'
                return 0.0, 0.0
            w = _clamp(-c['k_heading'] * _ang_norm(odom_yaw - self._hold_yaw),
                       c['w_max'])
            return c['v_approach'], w

        if self._cl == 'TURN2':
            err = _ang_norm(self._cl_yaw0 + self._th1 + self._th2 - odom_yaw)
            if abs(err) < c['turn_tol']:
                self.phase = 'APPROACHING'   # 재검출 + 미세정렬로 마무리
                self._lost_since = None
                return 0.0, 0.0
            return 0.0, _clamp(c['k_turn'] * err, c['turn_w'])

        return 0.0, 0.0


class DockServer(Node):
    def __init__(self, **kwargs):
        super().__init__('ddago_dock_server', **kwargs)
        self._cb = ReentrantCallbackGroup()

        self.declare_parameter('robot_id', 'dg_01')
        self.declare_parameter(
            'camera_calib_file', '/home/pinky/charuco_dock_ws/camera_calib.npz')
        self.declare_parameter('camera_width', 1280)
        self.declare_parameter('camera_height', 720)
        self.declare_parameter('odom_topic', 'odom')
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
        self.declare_parameter('rotate_180', True)
        self.declare_parameter('dry_run', False)
        for k, v in DEF.items():
            self.declare_parameter(k, v)

        self._robot_id = self.get_parameter('robot_id').value
        self._rotate_180 = bool(self.get_parameter('rotate_180').value)
        self._dry_run = bool(self.get_parameter('dry_run').value)
        self._res = (int(self.get_parameter('camera_width').value),
                     int(self.get_parameter('camera_height').value))

        # --- 상태 ---
        self._lock = threading.Lock()
        self._picam = None          # 도킹 중에만 연다(그 외엔 다른 노드가 쓸 수 있게)
        self._mtx = None            # 카메라 내부 파라미터
        self._dist = None
        self._odom_yaw = None
        self._odom_xy = None
        self._busy = threading.Lock()   # 동시 goal 방지(도킹은 배타적 자원)

        self._calib_path = self.get_parameter('camera_calib_file').value
        self._load_calib(self._calib_path)

        odom = self.get_parameter('odom_topic').value
        self.create_subscription(Odometry, odom, self._on_odom, 10,
                                 callback_group=self._cb)
        self._cmd_pub = self.create_publisher(
            Twist, self.get_parameter('cmd_vel_topic').value, 10)

        # 현재 task 알림 (로봇 내부 신호). telemetry_publisher 가 이 값을 텔레메트리의
        # task_id 로 싣는다 — 두 노드는 프로세스가 달라 변수를 공유할 수 없다.
        # 문서 규정: task_id = 로봇이 마지막으로 받은 Navigate/Dock goal 의 task_id.
        # navigate_server 와 같은 latched(TRANSIENT_LOCAL, depth 1) 조합이라
        # 구독자가 나중에 떠도 마지막 값을 받는다.
        self._task_pub = self.create_publisher(
            Int64, '/ddago/current_task',
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )

        self._server = ActionServer(
            self, Dock, '/ddago/dock',
            execute_callback=self._execute,
            cancel_callback=lambda _gh: CancelResponse.ACCEPT,
            callback_group=self._cb)

        self.get_logger().info(
            'Dock 서버 준비됨: robot_id=%s → /ddago/dock, 정면 picamera %dx%d%s, odom=%s%s'
            % (self._robot_id, self._res[0], self._res[1],
               '(180도 회전)' if self._rotate_180 else '', odom,
               '  ⚠️DRY-RUN(주행 명령 미발행)' if self._dry_run else ''))

    # ------------------------------------------------------------------ #
    def _load_calib(self, path):
        """내부파라미터(mtx,dist)를 npz 에서 읽는다. 캘리브 해상도와 캡처 해상도가
        다르면 초점거리 축척이 어긋나 거리(d)가 통째로 틀어지므로 함께 검사한다."""
        try:
            data = np.load(path)
            self._mtx, self._dist = data['mtx'], data['dist']
            msg = '카메라 캘리브 로드: %s (fx=%.1f)' % (path, self._mtx[0, 0])
            if 'resolution' in data:
                rw, rh = (int(v) for v in data['resolution'])
                if (rw, rh) != self._res:
                    self.get_logger().error(
                        '캘리브 해상도 %dx%d 와 캡처 %dx%d 가 다르다 — 거리 추정이 '
                        '틀어진다. camera_width/height 를 맞출 것' % (rw, rh, *self._res))
                else:
                    msg += ' %dx%d' % (rw, rh)
            self.get_logger().info(msg)
        except Exception as e:   # noqa: BLE001 - 파일 없음/형식 오류 모두 goal 에서 거절
            self._mtx = None
            self.get_logger().error('카메라 캘리브 로드 실패(%s): %s' % (path, e))

    # --- 정면 picamera(CSI) 직접 사용 ---------------------------------- #
    # 측면 순찰 웹캠(image_raw)과는 다른 장치라 서로 간섭하지 않는다.
    # 도킹 중에만 열고 끝나면 닫는다 — 다른 노드가 쓸 여지를 남기고, 장시간 점유로
    # 인한 파이프라인 정지(현장에서 겪은 영상 멈춤)의 노출도 줄인다.
    def _open_camera(self):
        from libcamera import Transform
        from picamera2 import Picamera2
        picam = Picamera2()
        tf = Transform(hflip=1, vflip=1) if self._rotate_180 else Transform()
        picam.configure(picam.create_video_configuration(
            main={'size': self._res, 'format': 'RGB888'}, transform=tf))
        picam.start()
        time.sleep(1.0)          # 노출/화이트밸런스 안정화
        self._picam = picam
        return picam

    def _close_camera(self):
        if self._picam is not None:
            try:
                self._picam.stop()
                self._picam.close()
            except Exception:    # noqa: BLE001 - 닫기 실패가 도킹 결과를 바꾸진 않는다
                pass
            self._picam = None

    def _on_odom(self, msg):
        p = msg.pose.pose.position
        with self._lock:
            self._odom_yaw = _yaw_from_quat(msg.pose.pose.orientation)
            self._odom_xy = (p.x, p.y)

    def _odom_snapshot(self):
        with self._lock:
            return self._odom_yaw, self._odom_xy

    def _publish(self, v, w):
        if self._dry_run:
            return
        m = Twist()
        m.linear.x = float(v)
        m.angular.z = float(w)
        self._cmd_pub.publish(m)

    def _stop(self, repeat=5):
        """정지 명령을 반복 발행. bringup 에 cmd_vel 워치독이 없어 마지막 명령이
        그대로 유지되므로, 어떤 경로로 끝나든 반드시 여기를 지나야 한다."""
        for _ in range(repeat):
            self._publish(0.0, 0.0)
            time.sleep(0.02)

    # ------------------------------------------------------------------ #
    def _cfg(self, goal):
        """파라미터 + goal 오버라이드를 상수 dict 로. goal 의 0 은 '기본값 사용'."""
        g = lambda k: self.get_parameter(k).value   # noqa: E731
        c = {
            'v_approach': float(g('v_approach')),
            'v_reverse': float(g('v_reverse')),
            'search_w': float(g('search_w')),
            'turn_w': float(g('turn_w')),
            'turn_tol': math.radians(float(g('turn_tol_deg'))),
            'k_turn': float(g('k_turn')),
            'k_bearing': float(g('k_bearing')),
            'k_heading': float(g('k_heading')),
            'w_max': float(g('w_max')),
            'bearing_tol': math.radians(float(g('bearing_tol_deg'))),
            'yaw_tol': math.radians(float(g('yaw_tol_deg'))),
            'face_timeout': float(g('face_timeout')),
            'lost_timeout': float(g('lost_timeout')),
            'plan_timeout': float(g('plan_timeout')),
            'n_plan': int(g('n_plan')),
            'n_stage_floor': 8,
        }
        c['search_timeout'] = (float(g('search_revs')) * 2 * math.pi
                               / max(c['search_w'], 1e-3))
        staging = float(g('staging_distance'))
        reverse = float(g('reverse_distance'))
        # dock_offset_x = 마커 기준 최종 정차 거리. 주면 후진량을 거기에 맞춘다.
        if float(goal.dock_offset_x) > 0.0:
            reverse = max(0.0, staging - float(goal.dock_offset_x))
        return c, staging, reverse

    def _execute(self, goal_handle):
        goal = goal_handle.request
        result = Dock.Result()

        if not self._busy.acquire(blocking=False):
            goal_handle.abort()
            result.result_code = RC_ALIGN_FAILED
            result.message = '다른 도킹이 진행 중이다'
            return result
        try:
            return self._run(goal_handle, goal, result)
        finally:
            # 어떤 경로로 빠지든 로봇을 세우고 카메라를 놓는다.
            self._stop()
            self._close_camera()
            self._busy.release()

    def _run(self, goal_handle, goal, result):
        log = self.get_logger()
        try:
            tracker = CharucoTracker(
                goal, self.get_parameter('min_corners').value)
        except ValueError as e:
            goal_handle.abort()
            result.result_code = RC_MARKER_NOT_FOUND
            result.message = 'Goal 마커 정보 오류: %s' % e
            log.error(result.message)
            return result

        if self._mtx is None:
            goal_handle.abort()
            result.result_code = RC_MARKER_NOT_FOUND
            result.message = ('카메라 내부파라미터 없음 — camera_calib_file 확인 (%s)'
                              % self._calib_path)
            log.error(result.message)
            return result

        try:
            picam = self._open_camera()
        except Exception as e:   # noqa: BLE001 - 카메라를 못 열면 도킹 자체가 불가
            goal_handle.abort()
            result.result_code = RC_MARKER_NOT_FOUND
            result.message = '정면 카메라 열기 실패: %s' % e
            log.error(result.message)
            return result

        cfg, staging, reverse = self._cfg(goal)
        fsm = DockFsm(cfg, staging, reverse, log=lambda m: log.info('[dock] %s' % m))
        period = 1.0 / max(float(self.get_parameter('control_hz').value), 1.0)
        log.info('도킹 시작: task=%d point=%s 보드=%dx%d id=%s staging=%.2fm 후진=%.2fm'
                 % (goal.task_id, goal.task_point_id, goal.squares_x,
                    goal.squares_y, goal.marker_id, staging, reverse))
        # goal 이 끝나도 0 으로 되돌리지 않는다: E4 복귀·도킹은 새 task 를 만들지 않고
        # 끝난 순찰의 task_id 를 그대로 쓰므로, 여기서 0 이 되면 '어느 작업 때문에
        # 복귀·도킹 중인지' 추적이 끊긴다.
        task_msg = Int64()
        task_msg.data = int(goal.task_id)
        self._task_pub.publish(task_msg)
        log.info('현재 task 알림 → /ddago/current_task: task_id=%d' % task_msg.data)

        fb = Dock.Feedback()
        last_fb = 0.0
        while rclpy.ok():
            t0 = time.monotonic()

            if goal_handle.is_cancel_requested:
                self._stop()
                goal_handle.canceled()
                result.result_code = RC_CANCELLED
                result.message = '취소됨'
                log.warn('[dock] 취소 요청 — 정지')
                return result

            # --- 관측 (정면 picamera 직접 캡처) ---
            found, obs, plan = False, None, None
            try:
                frame = picam.capture_array()
                # 180도 회전은 ISP(Transform)가 이미 처리했다 — CPU 회전 불필요.
                gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
            except Exception as e:   # noqa: BLE001 - 한 프레임 실패는 넘기고 계속
                log.warn('[dock] 캡처 실패, 계속: %s' % e)
                time.sleep(0.1)
                continue
            det = tracker.detect(gray, self._mtx, self._dist)
            if det is not None:
                d, bearing, yaw, n, rvec, tvec = det
                obs, found = (d, bearing, yaw, n), True
                plan = tracker.centerline_plan(rvec, tvec, staging)

            odom_yaw, odom_xy = self._odom_snapshot()
            v, w = fsm.update(time.monotonic(), found, obs, plan,
                              odom_yaw, odom_xy)
            self._publish(v, w)

            # --- 피드백 (5Hz 로 낮춰 DDS/CPU 부담을 줄인다) ---
            now = time.monotonic()
            if now - last_fb >= 0.2:
                fb.phase = fsm.phase
                fb.marker_detected = found
                fb.distance_to_marker_m = float(obs[0]) if found else 0.0
                goal_handle.publish_feedback(fb)
                last_fb = now

            if fsm.done:
                self._stop()
                self._fill_result(result, fsm)
                if fsm.result_code == RC_OK:
                    goal_handle.succeed()
                    log.info('[dock] 완료: %s' % result.message)
                else:
                    goal_handle.abort()
                    log.error('[dock] 실패(%d): %s'
                              % (fsm.result_code, result.message))
                return result

            time.sleep(max(0.0, period - (time.monotonic() - t0)))

        self._stop()
        goal_handle.abort()
        result.result_code = RC_CANCELLED
        result.message = '노드 종료'
        return result

    @staticmethod
    def _fill_result(result, fsm):
        """오차를 축별로 채운다. sigma = bearing - yaw 가 중심선 이탈각이고,
        이탈거리 ~= d*sin(sigma) 임이 실측으로 확인됐다(줄자 대비 mm 일치)."""
        result.result_code = fsm.result_code
        result.message = fsm.message
        b = fsm.last_bearing
        y = fsm.last_yaw
        d = fsm.last_d
        if b is None or y is None or d is None:
            return
        sigma = _ang_norm(b - y)
        result.final_lateral_m = float(d * math.sin(sigma))
        result.final_yaw_error = float(y)
        result.final_error_m = float(abs(result.final_lateral_m))


def main(args=None):
    # 노드를 만들기 전에 잡는다 — 늦게 잡으면 그 사이 goal 을 받을 수 있다.
    lock_fd, holder = acquire_single_instance()
    if lock_fd is None:
        print('[dock_server] 이미 실행 중이다 (PID %s). 중복 기동은 같은 액션 이름에\n'
              '              서버가 둘 생겨 goal 이 엉뚱한 쪽으로 갈 수 있어 막는다.\n'
              '              정리:  pkill -f dock_server' % holder, file=sys.stderr)
        sys.exit(1)

    rclpy.init(args=args)
    node = DockServer()
    # 액션 execute 가 루프를 도는 동안에도 카메라/odom 콜백이 계속 돌아야 한다.
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node._stop()          # 어떤 경로로 끝나도 로봇을 세운다
        node.destroy_node()
        rclpy.shutdown()
        os.close(lock_fd)     # 프로세스 종료 시 커널이 풀지만 명시적으로 닫는다


if __name__ == '__main__':
    main()
