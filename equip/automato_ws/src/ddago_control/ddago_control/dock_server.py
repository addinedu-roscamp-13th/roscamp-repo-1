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
import json
import math
import os
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from automato_interfaces.action import Dock
import cv2
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.logging import LoggingSeverity
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
    'plan_timeout': 8.0,        # 계획용 좋은 프레임 대기 상한(능동 후진 상한도 겸함)
    'plan_backup_max': 0.15,    # PLAN서 코너 부족 시 후진 한계 [m]. 보드가 바닥근처라
                                # 가까우면 아랫줄 코너가 화각을 벗어난다 → 살짝 후진해 넣는다
    'n_plan': 10,               # 계획 신뢰 최소 코너수(법선 정확도). 보드별로 낮출 수 있다
    'min_corners': 4,           # 검출 인정 최소 코너수
    'control_hz': 12.0,         # 제어 주기 — Pi4 CPU 과부하 방지
}

# 카메라 파이프라인 정지(프리즈) 대응. capture_array 가 예외 없이 영영 블록되는
# 현상을 겪었다(2026-07-23 ddago02, 커널 로그도 안 남음). 감지만 하고 **재오픈은
# 하지 않는다** — 멎은 드라이버를 다시 열면 dmabuf/CMA 가 물려 시스템 전체가 멎는다.
CAPTURE_TIMEOUT = 3.0          # 이 시간 안에 프레임이 안 오면 프리즈로 판정 [s]
STALL_STOP_SEC = 0.5           # 프레임이 이만큼 끊기면 워치독이 정지 발행 [s]

# 웹 스트리밍(디버그용). 기본 off — Pi4 에서 오버레이+JPEG 인코딩이 CPU 를 먹어
# 파이프라인 정지를 부를 수 있으므로 현장 확인이 필요할 때만 stream:=true 로 켠다.
STREAM_FPS = 15                # 스트림 송출 상한 [fps]

# 웹 수동 조그(원격조정 버튼). 도킹 goal 실행 중에는 무시된다(자율 제어 우선).
JOG_V = 0.06                   # 조그 선속 [m/s]
JOG_W = 0.4                    # 조그 각속 [rad/s]
JOG_EXPIRE = 0.4              # 데드맨 [s] — 이 시간 내 갱신 없으면 자동 정지


def _put(img, text, org, color):
    """검은 외곽선 위에 색 글자를 겹쳐 가독성을 높인다(dock_pose.py 와 동일)."""
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 4,
                cv2.LINE_AA)
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 1,
                cv2.LINE_AA)


# 웹 UI(dock_pose.py 스타일). 스트림 + 상태 + 조그패드 + 도킹취소.
# 도킹 '시작'은 dock_pose 와 달리 ROS 액션(ACS/send_goal)이 하므로 버튼이 없다.
DOCK_PAGE = """<!doctype html>
<html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>DdaGo Dock</title>
<style>
 body{font-family:sans-serif;margin:0;background:#111;color:#eee;text-align:center;
      -webkit-user-select:none;user-select:none}
 h1{font-size:1rem;margin:.4rem}
 #st{padding:.2rem;font-size:.95rem}
 #phase{padding:.2rem;font-size:1.15rem;font-weight:bold;color:#7cf}
 #vals{display:flex;justify-content:center;gap:.9rem;padding:.3rem;font-size:1.05rem;
       font-variant-numeric:tabular-nums;flex-wrap:wrap}
 #vals b{display:block;font-size:.7rem;color:#9ab;font-weight:normal}
 img{max-width:100%;height:auto;display:block;margin:0 auto;background:#000}
 #ctrl{background:#000;padding:.5rem;position:sticky;bottom:0}
 #jogwarn{font-size:.8rem;color:#fb6;min-height:1rem}
 .pad{display:grid;grid-template-columns:repeat(3,72px);grid-auto-rows:56px;gap:6px;
      justify-content:center;margin:.3rem 0 .5rem}
 .pad button{font-size:1.4rem;border:0;border-radius:10px;color:#fff;background:#357;
      touch-action:none}
 #fwd{grid-column:2} #lft{grid-column:1;grid-row:2}
 #stpj{grid-column:2;grid-row:2;background:#777}
 #rgt{grid-column:3;grid-row:2} #bwd{grid-column:2;grid-row:3}
 .runbtns{display:flex;gap:.5rem;justify-content:center}
 .runbtns button{flex:1;max-width:260px;padding:.8rem;font-size:1.05rem;border:0;
      border-radius:8px;color:#fff;background:#944}
</style></head>
<body>
<h1>DdaGo Dock <span id="mode"></span></h1>
<div id="st">connecting...</div>
<div id="phase">-</div>
<div id="vals">
 <div><span id="d">-</span><b>거리 d</b></div>
 <div><span id="brg">-</span><b>bearing</b></div>
 <div><span id="yaw">-</span><b>yaw</b></div>
 <div><span id="od">-</span><b>odom yaw</b></div>
 <div><span id="cmd">-</span><b>v / w</b></div>
</div>
<img src="/stream" alt="stream">
<div id="ctrl">
 <div id="jogwarn"></div>
 <div class="pad">
  <button id="fwd">▲</button>
  <button id="lft">◀</button>
  <button id="stpj">■</button>
  <button id="rgt">▶</button>
  <button id="bwd">▼</button>
 </div>
 <div class="runbtns"><button id="cancelb">■ 도킹 취소 / 정지</button></div>
</div>
<script>
 const $=i=>document.getElementById(i);
 const post=p=>fetch(p,{method:'POST'});
 function hold(btn,dir){
   let iv=null;
   const go=e=>{e.preventDefault(); post('/jog/'+dir);
     iv=iv||setInterval(()=>post('/jog/'+dir),150);};
   const end=e=>{if(e)e.preventDefault(); if(iv){clearInterval(iv);iv=null;} post('/jog/stop');};
   btn.addEventListener('mousedown',go); btn.addEventListener('touchstart',go,{passive:false});
   btn.addEventListener('mouseup',end); btn.addEventListener('mouseleave',end);
   btn.addEventListener('touchend',end); btn.addEventListener('touchcancel',end);
 }
 hold($('fwd'),'forward'); hold($('bwd'),'back');
 hold($('lft'),'left'); hold($('rgt'),'right');
 $('stpj').onclick=()=>post('/stop');
 $('cancelb').onclick=()=>{if(confirm('진행 중인 도킹을 취소/정지할까요?'))post('/cancel');};
 async function poll(){
   try{const r=await fetch('/status');const s=await r.json();
     $('mode').textContent=(s.motors?'⚠LIVE':'(dry-run)')+(s.busy?'  ●도킹중':'');
     $('mode').style.color=s.motors?'#fd6':'#9ab';
     $('st').textContent=s.busy?(s.found?`검출 OK | corners ${s.corners}`:'보드 미검출')
                                :'대기 (goal 없음)';
     $('st').style.color=s.busy?(s.found?'#6f6':'#f66'):'#9ab';
     $('phase').textContent=s.phase;
     $('jogwarn').textContent=s.busy?'도킹 중 — 수동 조그는 무시됩니다(취소 후 조작)':'';
     $('d').textContent=s.found?`${(s.d*100).toFixed(1)}cm`:'-';
     $('brg').textContent=s.found?`${s.bearing>=0?'+':''}${s.bearing.toFixed(1)}°`:'-';
     $('yaw').textContent=s.found?`${s.yaw>=0?'+':''}${s.yaw.toFixed(1)}°`:'-';
     $('od').textContent=(s.odom_yaw==null)?'--':`${s.odom_yaw>=0?'+':''}${s.odom_yaw.toFixed(1)}°`;
     $('cmd').textContent=`${s.v.toFixed(3)} / ${s.w.toFixed(3)}`;
   }catch(e){$('st').textContent='연결 끊김';}
 }
 setInterval(poll,300);poll();
</script>
</body></html>
"""


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
        self._cl_plan_xy0 = None   # PLAN 능동 후진 거리 측정 기준점
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
                self._cl_plan_xy0 = None
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
                self._cl_plan_xy0 = odom_xy   # 능동 후진 거리 측정 기준
            if not (found and plan is not None and obs[3] >= c['n_plan']):
                # 코너 부족 → 정지 대기는 무의미(장면이 안 변한다). 능동 대응:
                #  보드가 바닥근처라 가까우면 아랫줄 코너가 화각을 벗어난다.
                #  → staging 보다 가까우면 살짝 후진해 보드를 화면에 다 넣는다
                #    (bearing 은 유지해 중앙 정렬). 멀면 후진은 악화라 그냥 대기.
                n_now = obs[3] if obs is not None else 0
                backed = 0.0
                if odom_xy is not None and self._cl_plan_xy0 is not None:
                    backed = math.hypot(odom_xy[0] - self._cl_plan_xy0[0],
                                        odom_xy[1] - self._cl_plan_xy0[1])
                if (now - self._cl_since > c['plan_timeout']
                        or backed >= c['plan_backup_max']):
                    self._log('중심선 계획 실패(코너 %d/%d, %.0fcm 후진) → 접근으로 폴백'
                              % (n_now, c['n_plan'], backed * 100))
                    self.phase = 'APPROACHING'
                    return 0.0, 0.0
                if (found and obs[0] < self.staging + c['plan_backup_max']
                        and odom_xy is not None):
                    return -c['v_approach'], _clamp(-c['k_bearing'] * obs[1],
                                                    c['w_max'])
                return 0.0, 0.0              # 멀거나 미검출·odom없음 → 대기
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
        # 디버그: 매 제어주기 상세(phase·d·b·y·n·v·w·fps)를 DEBUG 로그로. dock_pose.py
        # 터미널 상태줄에 해당. 기본 off — 켜면 노드 로거 레벨을 DEBUG 로 올린다.
        self.declare_parameter('debug', False)
        # 웹 스트리밍: 오버레이한 카메라 화면을 MJPEG 로. 제어는 ROS 액션이 하므로
        # dock_pose.py 와 달리 **뷰 전용**(조작 버튼 없음). 기본 off(Pi4 부하).
        self.declare_parameter('stream', False)
        self.declare_parameter('stream_port', 8000)
        self.declare_parameter('stream_quality', 80)
        for k, v in DEF.items():
            self.declare_parameter(k, v)

        self._robot_id = self.get_parameter('robot_id').value
        self._rotate_180 = bool(self.get_parameter('rotate_180').value)
        self._dry_run = bool(self.get_parameter('dry_run').value)
        self._res = (int(self.get_parameter('camera_width').value),
                     int(self.get_parameter('camera_height').value))
        self._debug = bool(self.get_parameter('debug').value)
        if self._debug:
            self.get_logger().set_level(LoggingSeverity.DEBUG)

        # --- 상태 ---
        self._lock = threading.Lock()
        self._camera_wedged = False  # 프리즈 겪으면 True → 재오픈 금지(노드 재시작까지)

        # 카메라 공유(참조카운트): 웹 뷰어(/stream) 또는 도킹 goal 이 카메라를 '점유'
        # 하면 캡처 스레드가 돈다. 둘 다 없으면 카메라를 닫아 다른 노드에 양보한다.
        # 캡처는 이 한 스레드만 하고 goal 루프·스트림은 공유 버퍼를 읽는다 — goal 루프가
        # 캡처에 블록되지 않으므로 취소가 항상 동작한다(프리즈 안전).
        self._picam = None
        self._cam_lock = threading.Lock()
        self._cam_cv = threading.Condition(self._cam_lock)
        self._cam_users = 0
        self._frame = None          # 최신 raw RGB 프레임(공유 버퍼)
        self._frame_t = 0.0         # 그 프레임 캡처 시각(monotonic) — 프리즈 판정

        # 웹 스트리밍
        self._jpeg_lock = threading.Lock()
        self._latest_jpeg = None
        self._stream_enc = [int(cv2.IMWRITE_JPEG_QUALITY),
                            int(self.get_parameter('stream_quality').value)]
        self._stream = bool(self.get_parameter('stream').value)
        self._stream_port = int(self.get_parameter('stream_port').value)
        self._httpd = None

        # 웹 상태표시(/status)용 최신 스냅샷 (각도 degree, 거리 m)
        self._status = {
            'phase': 'IDLE', 'found': False, 'corners': 0, 'd': 0.0,
            'bearing': 0.0, 'yaw': 0.0, 'odom_yaw': None, 'v': 0.0, 'w': 0.0,
            'fps': 0.0, 'message': '', 'motors': not self._dry_run, 'busy': False}

        # 웹 수동 조그 (v, w, expire) 또는 None. goal 중엔 무시된다.
        self._manual = None
        self._active_goal = False        # 도킹 goal 실행 중?
        self._web_cancel = threading.Event()   # 웹 '도킹 취소' 버튼

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

        # 공유 카메라 캡처 스레드(사용자 0이면 대기). goal·스트림 공용.
        threading.Thread(target=self._camera_loop, daemon=True).start()
        # 웹 수동 조그 발행(10Hz). goal 중이거나 조그 없으면 아무 것도 안 한다.
        self.create_timer(0.1, self._manual_tick, callback_group=self._cb)
        if self._stream:
            self._start_stream_server()

        self.get_logger().info(
            'Dock 서버 준비됨: robot_id=%s → /ddago/dock, 정면 picamera %dx%d%s, odom=%s%s%s%s'
            % (self._robot_id, self._res[0], self._res[1],
               '(180도 회전)' if self._rotate_180 else '', odom,
               '  ⚠️DRY-RUN(주행 명령 미발행)' if self._dry_run else '',
               '  DEBUG로그' if self._debug else '',
               ('  스트림 http://<ip>:%d' % self._stream_port) if self._stream else ''))

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
    # 측면 순찰 웹캠(image_raw)과는 다른 장치라 서로 간섭하지 않는다. 캡처 스레드가
    # 참조카운트(_cam_users)로 소유 — 도킹 goal 또는 웹 뷰어가 있을 때만 열고, 둘 다
    # 없으면 닫아 다른 노드에 양보한다(장시간 점유로 인한 파이프라인 정지 노출도 줄임).
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
        # 참조를 먼저 버린다: 멎은 드라이버에선 stop()/close() 자체가 블록될 수 있어
        # (2026-07-23 실측), 그걸 별도 스레드로 돌려 execute 콜백이 안 막히게 한다.
        # 블록돼도 결과는 이미 반환되고, 재오픈은 어차피 _camera_wedged 가 막는다.
        picam, self._picam = self._picam, None
        if picam is None:
            return

        def _shut():
            try:
                picam.stop()
                picam.close()
            except Exception:    # noqa: BLE001 - 닫기 실패가 도킹 결과를 바꾸진 않는다
                pass

        t = threading.Thread(target=_shut, daemon=True)
        t.start()
        t.join(timeout=2.0)      # 정상이면 즉시 끝남 / 멎었으면 상한만 기다리고 넘어감

    # --- 공유 캡처 스레드(참조카운트) --------------------------------- #
    def _cam_acquire(self):
        with self._cam_cv:
            self._cam_users += 1
            self._cam_cv.notify_all()

    def _cam_release(self):
        with self._cam_cv:
            self._cam_users = max(0, self._cam_users - 1)
            self._cam_cv.notify_all()

    def _frame_snapshot(self):
        with self._cam_lock:
            return self._frame, self._frame_t

    def _camera_loop(self):
        """사용자(웹 뷰어/goal)가 있으면 카메라를 열고 계속 캡처해 공유 버퍼를 채운다.
        사용자가 0이 되면 닫아 다른 노드에 양보한다. 프리즈를 겪으면 재오픈하지 않는다."""
        while rclpy.ok():
            with self._cam_cv:
                while rclpy.ok() and self._cam_users == 0:
                    self._cam_cv.wait(timeout=0.5)
            if not rclpy.ok():
                return
            if self._camera_wedged:          # 재오픈 금지 — 잠깐 쉬고 다시 대기
                time.sleep(0.5)
                continue
            try:
                picam = self._open_camera()
            except Exception as e:           # noqa: BLE001
                self.get_logger().error('정면 카메라 열기 실패: %s' % e)
                time.sleep(1.0)
                continue
            try:
                self._capture_until_idle(picam)
            finally:
                self._close_camera()

    def _capture_until_idle(self, picam):
        """사용자가 있는 동안 계속 캡처. 프리즈(capture 타임아웃)면 정지 발행 후 종료."""
        fps_t0, fps_n = time.monotonic(), 0
        while rclpy.ok():
            with self._cam_lock:
                if self._cam_users == 0:
                    return
            try:
                # 타임아웃 캡처라야 파이프라인 정지를 인지하고 빠져나올 수 있다.
                job = picam.capture_array(wait=False)
                frame = picam.wait(job, timeout=CAPTURE_TIMEOUT)
            except TimeoutError:
                self._camera_wedged = True
                self._stop()
                self.get_logger().error(
                    '카메라 파이프라인 정지(프리즈) — 정지 발행, 재오픈 금지. '
                    '노드 재시작 필요')
                return
            except Exception as e:           # noqa: BLE001 - 한 프레임 실패는 넘기고 계속
                self.get_logger().warn('캡처 실패, 계속: %s' % e)
                time.sleep(0.1)
                continue
            now = time.monotonic()
            with self._cam_lock:
                self._frame = frame
                self._frame_t = now
            fps_n += 1
            if now - fps_t0 >= 1.0:
                with self._lock:
                    self._status['fps'] = fps_n / (now - fps_t0)
                fps_t0, fps_n = now, 0
            if self._stream:
                self._render_stream(frame)

    def _render_stream(self, frame):
        """공유 버퍼 프레임에 최신 상태(self._status)를 오버레이해 JPEG 로 만든다."""
        with self._lock:
            st = dict(self._status)
        vis = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        if st['found']:
            line = ('%s  d=%5.1fcm b=%+5.1f y=%+5.1f n=%d'
                    % (st['phase'], st['d'] * 100, st['bearing'], st['yaw'],
                       st['corners']))
            col = (0, 255, 0)
        else:
            line = '%s  board not found' % st['phase']
            col = (0, 200, 255) if st['busy'] else (200, 200, 200)
        oy = '--' if st['odom_yaw'] is None else '%+.1f' % st['odom_yaw']
        _put(vis, line, (10, 26), col)
        _put(vis, 'v=%+.3f w=%+.3f odom=%s %.0ffps%s'
             % (st['v'], st['w'], oy, st['fps'],
                '  DRY-RUN' if self._dry_run else ''),
             (10, 52), (0, 200, 255))
        ok, buf = cv2.imencode('.jpg', vis, self._stream_enc)
        if ok:
            with self._jpeg_lock:
                self._latest_jpeg = buf.tobytes()

    def _manual_tick(self):
        """웹 수동 조그를 주기 발행(데드맨). 도킹 goal 중엔 무시(자율 제어 우선)."""
        if self._active_goal:
            return
        m = self._manual
        if m is None:
            return
        v, w, exp = m
        if time.monotonic() > exp:        # 데드맨 만료 → 정지
            self._manual = None
            self._publish(0.0, 0.0)
            return
        self._publish(v, w)

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

    # --- 웹 (스트림 + 상태 + 원격조정) -------------------------------- #
    def _start_stream_server(self):
        """오버레이 카메라 화면(MJPEG) + 상태(/status) + 조작(/jog·/stop·/cancel)
        을 내는 HTTP 서버를 데몬 스레드로. 웹 접속(/stream) 시 카메라를 점유해
        goal 없이도 라이브로 보인다. 조그는 goal 중엔 무시된다."""
        handler = self._make_web_handler()
        try:
            self._httpd = ThreadingHTTPServer(('0.0.0.0', self._stream_port), handler)
        except OSError as e:     # 포트 점유 등 — 웹만 포기하고 도킹은 계속
            self.get_logger().warn('웹 서버 기동 실패(계속): %s' % e)
            self._httpd = None
            return
        threading.Thread(target=self._httpd.serve_forever, daemon=True).start()
        self.get_logger().info('웹: http://<ip>:%d/' % self._stream_port)

    def _make_web_handler(self):
        server = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *a):        # 접속 로그 억제
                pass

            def _send(self, body, ctype):
                self.send_response(200)
                self.send_header('Content-Type', ctype)
                self.send_header('Content-Length', str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def do_GET(self):
                if self.path == '/':
                    self._send(DOCK_PAGE.encode('utf-8'),
                               'text/html; charset=utf-8')
                elif self.path == '/status':
                    with server._lock:
                        data = dict(server._status)
                    self._send(json.dumps(data).encode(), 'application/json')
                elif self.path == '/stream':
                    server._cam_acquire()     # 웹 접속 → 카메라 상시 라이브
                    self.send_response(200)
                    self.send_header(
                        'Content-Type',
                        'multipart/x-mixed-replace; boundary=frame')
                    self.end_headers()
                    try:
                        while rclpy.ok():
                            with server._jpeg_lock:
                                jpg = server._latest_jpeg
                            if jpg:
                                self.wfile.write(b'--frame\r\n')
                                self.wfile.write(b'Content-Type: image/jpeg\r\n')
                                self.wfile.write(
                                    ('Content-Length: %d\r\n\r\n'
                                     % len(jpg)).encode())
                                self.wfile.write(jpg)
                                self.wfile.write(b'\r\n')
                            time.sleep(1.0 / STREAM_FPS)
                    except (BrokenPipeError, ConnectionResetError):
                        pass
                    finally:
                        server._cam_release()  # 뷰어 떠나면 카메라 양보
                else:
                    self.send_error(404)

            def do_POST(self):
                if self.path.startswith('/jog/'):
                    key = self.path.rsplit('/', 1)[1]
                    vmap = {'forward': (JOG_V, 0.0), 'back': (-JOG_V, 0.0),
                            'left': (0.0, JOG_W), 'right': (0.0, -JOG_W),
                            'stop': (0.0, 0.0)}
                    if key not in vmap:
                        self.send_error(404)
                        return
                    if key == 'stop':
                        server._manual = None
                        if not server._active_goal:
                            server._publish(0.0, 0.0)
                    else:
                        v, w = vmap[key]
                        server._manual = (v, w, time.monotonic() + JOG_EXPIRE)
                    self._send(b'', 'text/plain')
                elif self.path == '/stop':
                    server._manual = None
                    if not server._active_goal:
                        server._publish(0.0, 0.0)
                    self._send(b'', 'text/plain')
                elif self.path == '/cancel':
                    server._web_cancel.set()  # goal 루프가 다음 주기에 처리
                    self._send(b'', 'text/plain')
                else:
                    self.send_error(404)

        return Handler

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
            'plan_backup_max': float(g('plan_backup_max')),
            'n_stage_floor': 8,
        }
        # 계획 요구 코너수를 이 보드에 맞춘다. 4x4 는 최대 9개라 기본 10 을 못 넘어
        # 계획이 영영 안 잡힌다 → min(n_plan, (sx-1)*(sy-1)-1) 로 낮춘다(4x4=8).
        sx, sy = int(goal.squares_x), int(goal.squares_y)
        c['n_plan'] = min(int(g('n_plan')), (sx - 1) * (sy - 1) - 1)
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
            # 어떤 경로로 빠지든 로봇을 세운다. 카메라는 캡처 스레드가 소유하므로
            # 여기서 닫지 않는다(_run 이 _cam_release 로 사용자 카운트를 내린다).
            self._stop()
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

        if self._camera_wedged:
            goal_handle.abort()
            result.result_code = RC_ALIGN_FAILED
            result.message = ('이전 도킹에서 카메라 프리즈 발생 — 재오픈 시 시스템 정지 '
                              '위험이라 도킹 비활성. 노드 재시작 필요')
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

        # 공유 카메라 점유(캡처 스레드가 프레임을 채운다) + 웹 상태/취소 초기화.
        self._web_cancel.clear()
        self._active_goal = True
        with self._lock:
            self._status['busy'] = True
        self._cam_acquire()

        fb = Dock.Feedback()
        last_fb = 0.0
        last_dbg = 0.0
        # 신선한 프레임을 마지막으로 받은 시각(공유 캡처 스레드 상태 감시용).
        fresh_t = time.monotonic()
        try:
            while rclpy.ok():
                t0 = time.monotonic()

                # 취소: 액션 클라이언트 취소 or 웹 '취소' 버튼.
                if goal_handle.is_cancel_requested or self._web_cancel.is_set():
                    self._stop()
                    result.result_code = RC_CANCELLED
                    if goal_handle.is_cancel_requested:
                        goal_handle.canceled()
                        result.message = '취소됨'
                    else:
                        goal_handle.abort()      # 웹 취소는 액션 취소 절차가 아님
                        result.message = '웹 취소'
                    log.warn('[dock] 취소 — 정지 (%s)' % result.message)
                    return result

                # --- 공유 버퍼에서 최신 프레임 ---
                # 캡처는 별도 스레드가 하므로 여기서 블록되지 않는다(취소 항상 가능).
                frame, frame_t = self._frame_snapshot()
                now = time.monotonic()
                if frame is None or (now - frame_t) > STALL_STOP_SEC:
                    # 프레임 끊김 → 즉시 정지(프리즈 순간 속도 유지 방지).
                    self._stop()
                    if self._camera_wedged or (now - fresh_t) > CAPTURE_TIMEOUT + 2.0:
                        goal_handle.abort()
                        result.result_code = RC_ALIGN_FAILED
                        result.message = ('카메라 프레임 정지 — 도킹 중단'
                                          + ('(프리즈, 노드 재시작 필요)'
                                             if self._camera_wedged else ''))
                        log.error('[dock] %s' % result.message)
                        return result
                    time.sleep(0.03)
                    continue
                fresh_t = now

                found, obs, plan = False, None, None
                gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
                det = tracker.detect(gray, self._mtx, self._dist)
                if det is not None:
                    d, bearing, yaw, n, rvec, tvec = det
                    obs, found = (d, bearing, yaw, n), True
                    plan = tracker.centerline_plan(rvec, tvec, staging)

                odom_yaw, odom_xy = self._odom_snapshot()
                v, w = fsm.update(time.monotonic(), found, obs, plan,
                                  odom_yaw, odom_xy)
                self._publish(v, w)

                # 웹 상태표시(/status·스트림 오버레이) 갱신
                with self._lock:
                    self._status.update(
                        phase=fsm.phase, found=found,
                        corners=(obs[3] if found else 0),
                        d=(obs[0] if found else 0.0),
                        bearing=(math.degrees(obs[1]) if found else 0.0),
                        yaw=(math.degrees(obs[2]) if found else 0.0),
                        odom_yaw=(None if odom_yaw is None
                                  else math.degrees(odom_yaw)),
                        v=v, w=w, message=fsm.message)

                # DEBUG 로그: 상세 상태를 0.5s 스로틀로. dock_pose.py 상태줄 대응.
                if self._debug and now - last_dbg >= 0.5:
                    oy = ('%+.1f' % math.degrees(odom_yaw)
                          if odom_yaw is not None else '--')
                    if found:
                        log.debug('[%s] d=%5.1fcm b=%+5.1f y=%+5.1f n=%d odom=%s '
                                  'v=%+.3f w=%+.3f'
                                  % (fsm.phase, obs[0] * 100, math.degrees(obs[1]),
                                     math.degrees(obs[2]), obs[3], oy, v, w))
                    else:
                        log.debug('[%s] board not found odom=%s v=%+.3f w=%+.3f'
                                  % (fsm.phase, oy, v, w))
                    last_dbg = now

                # --- 피드백 (5Hz 로 낮춰 DDS/CPU 부담을 줄인다) ---
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
        finally:
            self._active_goal = False
            with self._lock:
                self._status['busy'] = False
                self._status['phase'] = 'IDLE'
            self._cam_release()      # 카메라 사용자 카운트 감소(뷰어 없으면 닫힘)

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
