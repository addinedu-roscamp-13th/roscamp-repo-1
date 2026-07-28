#!/usr/bin/env python3
"""RP-126: DdaGo(주행 로봇) 바닥 H 마커 정밀 후진 도킹 — FloorDock Action 서버.

ChArUco 도킹(dock_server.py)과 별개 방식이다. 바닥에 그린 **청색 H자 테이프**를
전면 카메라로 보고 후면을 벽에 붙인다(마커리스, 바닥평면 homography).

  * FloorDock 액션의 **서버**   (DCS → DdaGo, /ddago/floor_dock)
  * 정면 picamera(CSI) **직접 사용** (측면 순찰 웹캠과 별개 장치)
  * odom 구독(개루프 기동 기준) / cmd_vel 발행

구조·안전은 dock_server.py 를 미러링:
  - 공유 카메라 캡처 스레드(참조카운트, 타임아웃 캡처 → 프리즈 감지 시 정지·재오픈 금지)
  - 종료·취소·실패 어느 경로로 빠져도 0 속도 반복(_stop). dry_run:=true 로 명령만 확인 가능.
  - 단일 인스턴스 파일 락(같은 액션 서버 둘 뜨는 사고 방지)

검출·FSM 은 floor_dock_ws(ddago01 실주행 검증본)에서 이식:
  floor_dock.floor_detector  : FloorMapper(floor_calib.npz) + find_dock(청색 H)
  floor_dock.floor_fsm       : SEARCH→CENTERLINE→ALIGN/FACE→STAGED→TURN→REVERSE(벽기준 동적)

⚠️ 첫 현장 투입은 dry_run:=true 로 명령만 확인 후 실주행(bringup 에 cmd_vel 워치독 없음).
"""
import fcntl
import math
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.logging import LoggingSeverity
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Int64

from automato_interfaces.action import FloorDock

from .floor_dock import floor_detector as detector
from .floor_dock import floor_fsm as fsm_mod

LOCK_PATH = os.environ.get('DDAGO_FLOOR_DOCK_LOCK', '/tmp/ddago_floor_dock_server.lock')

# ---- Result.result_code (FloorDock.action 주석과 일치) ----
RC_OK = 0
RC_MARKER_NOT_FOUND = 1
RC_TOLERANCE = 2
RC_CANCELLED = 3
RC_ALIGN_FAILED = 4

# 카메라 프리즈 대응(dock_server 와 동일): 감지만 하고 재오픈 금지.
CAPTURE_TIMEOUT = 3.0
STALL_STOP_SEC = 0.5
STREAM_FPS = 15


def acquire_single_instance(path=LOCK_PATH):
    """단일 인스턴스 파일 락. (fd, None) 성공 / (None, holder) 실패."""
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


def _put(img, text, org, color):
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 4, cv2.LINE_AA)
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 1, cv2.LINE_AA)


PAGE = """<!doctype html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>DdaGo Floor Dock</title>
<style>body{margin:0;background:#111;color:#eee;font-family:sans-serif;text-align:center}
img{max-width:100%;height:auto;background:#000}</style></head>
<body><h3>DdaGo Floor Dock (view only)</h3><img src="/stream" alt="stream"></body></html>"""


class FloorDockServer(Node):
    def __init__(self, **kwargs):
        super().__init__('ddago_floor_dock_server', **kwargs)
        self._cb = ReentrantCallbackGroup()

        self.declare_parameter('robot_id', 'dg_01')
        self.declare_parameter('floor_calib_file',
                               '/home/pinky/floor_dock_ws/floor_calib.npz')
        self.declare_parameter('camera_width', 1280)
        self.declare_parameter('camera_height', 720)
        self.declare_parameter('odom_topic', 'odom')
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
        self.declare_parameter('rotate_180', True)
        self.declare_parameter('dry_run', False)
        self.declare_parameter('debug', False)
        self.declare_parameter('stream', False)
        self.declare_parameter('stream_port', 8001)
        self.declare_parameter('stream_quality', 80)
        self.declare_parameter('control_hz', 12.0)
        # 튜닝(floor_fsm 로 configure). 기본은 floor_fsm 모듈 상수(ddago01 검증값).
        self.declare_parameter('d_stage', float(fsm_mod.D_STAGE))
        self.declare_parameter('wall_gap_target', float(fsm_mod.WALL_GAP_TARGET))
        self.declare_parameter('reverse_k', float(fsm_mod.REVERSE_K))
        self.declare_parameter('crossbar_to_wall', float(fsm_mod.CROSSBAR_TO_WALL))
        self.declare_parameter('lateral_offset', float(fsm_mod.LATERAL_OFFSET))
        self.declare_parameter('dynamic_reverse', bool(fsm_mod.DYNAMIC_REVERSE))
        self.declare_parameter('stage_settle_sec', float(fsm_mod.STAGE_SETTLE_SEC))
        self.declare_parameter('cl_verify_d', float(fsm_mod.CL_VERIFY_D))
        self.declare_parameter('cl_max_replans', int(fsm_mod.CL_MAX_REPLANS))
        # 도킹 후 후퇴 [m]. 0=벽에 붙어 유지(실배포). >0=반복 테스트용(다음 마커 보려 후퇴).
        self.declare_parameter('post_advance_m', 0.0)
        # 반복 시 도킹 완료 후 정지 유지 [s] (post_advance_m>0 경로에서만).
        self.declare_parameter('post_dock_hold_sec', float(fsm_mod.POST_DOCK_HOLD_SEC))

        self._robot_id = self.get_parameter('robot_id').value
        self._rotate_180 = bool(self.get_parameter('rotate_180').value)
        self._dry_run = bool(self.get_parameter('dry_run').value)
        self._res = (int(self.get_parameter('camera_width').value),
                     int(self.get_parameter('camera_height').value))
        self._debug = bool(self.get_parameter('debug').value)
        if self._debug:
            self.get_logger().set_level(LoggingSeverity.DEBUG)
        self._crossbar_to_wall = float(self.get_parameter('crossbar_to_wall').value)

        # FSM 기본 튜닝을 파라미터로 덮어쓴다(goal 이 다시 덮을 수 있다).
        self._apply_fsm_params()

        # 바닥 평면 캘리브(mtx/dist 내장). 실패하면 goal 을 거절한다.
        self._mapper = None
        self._calib_path = self.get_parameter('floor_calib_file').value
        self._load_calib(self._calib_path)

        # 카메라 공유(참조카운트) — dock_server 와 동일 구조.
        self._picam = None
        self._camera_wedged = False
        self._cam_lock = threading.Lock()
        self._cam_cv = threading.Condition(self._cam_lock)
        self._cam_users = 0
        self._frame = None
        self._frame_t = 0.0

        self._jpeg_lock = threading.Lock()
        self._latest_jpeg = None
        self._stream_enc = [int(cv2.IMWRITE_JPEG_QUALITY),
                            int(self.get_parameter('stream_quality').value)]
        self._stream = bool(self.get_parameter('stream').value)
        self._stream_port = int(self.get_parameter('stream_port').value)
        self._httpd = None

        self._odom_lock = threading.Lock()
        self._odom_yaw = None
        self._odom_xy = None
        self._busy = threading.Lock()

        odom = self.get_parameter('odom_topic').value
        self.create_subscription(Odometry, odom, self._on_odom, 10,
                                 callback_group=self._cb)
        self._cmd_pub = self.create_publisher(
            Twist, self.get_parameter('cmd_vel_topic').value, 10)
        # 현재 task 알림(telemetry_publisher 가 싣는다). dock_server 와 동일 QoS.
        self._task_pub = self.create_publisher(
            Int64, '/ddago/current_task',
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))

        self._server = ActionServer(
            self, FloorDock, '/ddago/floor_dock',
            execute_callback=self._execute,
            cancel_callback=lambda _gh: CancelResponse.ACCEPT,
            callback_group=self._cb)

        threading.Thread(target=self._camera_loop, daemon=True).start()
        if self._stream:
            self._start_stream_server()

        self.get_logger().info(
            'FloorDock 서버 준비됨: robot_id=%s → /ddago/floor_dock, 정면 picamera %dx%d%s, '
            'odom=%s%s%s%s' % (
                self._robot_id, self._res[0], self._res[1],
                '(180도 회전)' if self._rotate_180 else '', odom,
                '  ⚠️DRY-RUN' if self._dry_run else '',
                '  DEBUG' if self._debug else '',
                ('  스트림 http://<ip>:%d' % self._stream_port) if self._stream else ''))

    # ------------------------------------------------------------------ #
    def _apply_fsm_params(self, wall_gap=None, lateral=None):
        """노드 파라미터(+goal 오버라이드)를 floor_fsm 모듈 상수로 반영."""
        g = self.get_parameter
        fsm_mod.configure(
            D_STAGE=float(g('d_stage').value),
            WALL_GAP_TARGET=(wall_gap if wall_gap else float(g('wall_gap_target').value)),
            REVERSE_K=float(g('reverse_k').value),
            CROSSBAR_TO_WALL=float(g('crossbar_to_wall').value),
            LATERAL_OFFSET=(lateral if lateral is not None
                            else float(g('lateral_offset').value)),
            DYNAMIC_REVERSE=bool(g('dynamic_reverse').value),
            STAGE_SETTLE_SEC=float(g('stage_settle_sec').value),
            CL_VERIFY_D=float(g('cl_verify_d').value),
            CL_MAX_REPLANS=int(g('cl_max_replans').value),
            POST_DOCK_HOLD_SEC=float(g('post_dock_hold_sec').value))

    def _load_calib(self, path):
        """floor_calib.npz(바닥 평면 + 내장 mtx/dist) 로드. 실패하면 goal 거절."""
        try:
            self._mapper = detector.FloorMapper(path)
            self.get_logger().info(
                '바닥 캘리브 로드: %s (fx=%.1f)' % (path, self._mapper.mtx[0, 0]))
        except Exception as e:   # noqa: BLE001
            self._mapper = None
            self.get_logger().error('바닥 캘리브 로드 실패(%s): %s' % (path, e))

    # --- 정면 picamera(CSI) 공유 캡처 (dock_server 미러) ---------------- #
    def _open_camera(self):
        from libcamera import Transform
        from picamera2 import Picamera2
        picam = Picamera2()
        tf = Transform(hflip=1, vflip=1) if self._rotate_180 else Transform()
        picam.configure(picam.create_video_configuration(
            main={'size': self._res, 'format': 'RGB888'}, transform=tf))
        picam.start()
        time.sleep(1.0)
        self._picam = picam
        return picam

    def _close_camera(self):
        picam, self._picam = self._picam, None
        if picam is None:
            return

        def _shut():
            try:
                picam.stop()
                picam.close()
            except Exception:    # noqa: BLE001
                pass
        t = threading.Thread(target=_shut, daemon=True)
        t.start()
        t.join(timeout=2.0)

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
        while rclpy.ok():
            with self._cam_cv:
                while rclpy.ok() and self._cam_users == 0:
                    self._cam_cv.wait(timeout=0.5)
            if not rclpy.ok():
                return
            if self._camera_wedged:
                time.sleep(0.5)
                continue
            try:
                picam = self._open_camera()
            except Exception as e:   # noqa: BLE001
                self.get_logger().error('정면 카메라 열기 실패: %s' % e)
                time.sleep(1.0)
                continue
            try:
                self._capture_until_idle(picam)
            finally:
                self._close_camera()

    def _capture_until_idle(self, picam):
        while rclpy.ok():
            with self._cam_lock:
                if self._cam_users == 0:
                    return
            try:
                job = picam.capture_array(wait=False)
                frame = picam.wait(job, timeout=CAPTURE_TIMEOUT)
            except TimeoutError:
                self._camera_wedged = True
                self._stop()
                self.get_logger().error('카메라 파이프라인 정지(프리즈) — 정지 발행, '
                                        '재오픈 금지. 노드 재시작 필요')
                return
            except Exception as e:   # noqa: BLE001
                self.get_logger().warn('capture 예외(계속): %s' % e)
                time.sleep(0.1)
                continue
            with self._cam_lock:
                self._frame = frame          # picamera 'RGB888' 은 실제 BGR 배열
                self._frame_t = time.monotonic()

    # --- odom / cmd_vel ------------------------------------------------ #
    def _on_odom(self, msg):
        q = msg.pose.pose.orientation
        p = msg.pose.pose.position
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        with self._odom_lock:
            self._odom_yaw = yaw
            self._odom_xy = (p.x, p.y)

    def _odom_snapshot(self):
        with self._odom_lock:
            return self._odom_yaw, self._odom_xy

    def _publish(self, v, w):
        if self._dry_run:
            return
        m = Twist()
        m.linear.x = float(v)
        m.angular.z = float(w)
        self._cmd_pub.publish(m)

    def _stop(self):
        if self._dry_run:
            return
        m = Twist()
        for _ in range(3):           # 놓치면 계속 가므로 여러 번
            self._cmd_pub.publish(m)

    # --- 액션 실행 ----------------------------------------------------- #
    def _execute(self, goal_handle):
        result = FloorDock.Result()
        if not self._busy.acquire(blocking=False):
            goal_handle.abort()
            result.result_code = RC_ALIGN_FAILED
            result.message = '다른 도킹이 진행 중이다'
            return result
        try:
            return self._run(goal_handle, goal_handle.request, result)
        finally:
            self._stop()
            self._busy.release()

    def _run(self, goal_handle, goal, result):
        log = self.get_logger()
        if self._mapper is None:
            goal_handle.abort()
            result.result_code = RC_MARKER_NOT_FOUND
            result.message = '바닥 캘리브 없음 — floor_calib_file 확인 (%s)' % self._calib_path
            log.error(result.message)
            return result
        if self._camera_wedged:
            goal_handle.abort()
            result.result_code = RC_ALIGN_FAILED
            result.message = '이전 도킹서 카메라 프리즈 — 도킹 비활성(노드 재시작 필요)'
            log.error(result.message)
            return result

        # goal 오버라이드(0 이면 노드 기본): wall_gap_m, lateral_offset_m
        wall_gap = float(goal.wall_gap_m) if float(goal.wall_gap_m) > 0.0 else None
        lateral = float(goal.lateral_offset_m) if float(goal.lateral_offset_m) != 0.0 else None
        self._apply_fsm_params(wall_gap=wall_gap, lateral=lateral)

        fsm = fsm_mod.DockFsm()      # auto=True, use_centerline=True 기본
        fsm.post_advance_m = float(self.get_parameter('post_advance_m').value)
        period = 1.0 / max(float(self.get_parameter('control_hz').value), 1.0)
        log.info('바닥 H 도킹 시작: task=%d point=%s wall_gap=%s lateral=%s'
                 % (goal.task_id, goal.task_point_id,
                    ('%.3f' % wall_gap) if wall_gap else 'default',
                    ('%.3f' % lateral) if lateral is not None else 'default'))
        task_msg = Int64()
        task_msg.data = int(goal.task_id)
        self._task_pub.publish(task_msg)

        self._cam_acquire()
        fb = FloorDock.Feedback()
        last_fb = last_dbg = 0.0
        fresh_t = time.monotonic()
        try:
            while rclpy.ok():
                t0 = time.monotonic()
                if goal_handle.is_cancel_requested:
                    self._stop()
                    goal_handle.canceled()
                    result.result_code = RC_CANCELLED
                    result.message = '취소됨'
                    log.warn('[floor_dock] 취소 — 정지')
                    return result

                frame, frame_t = self._frame_snapshot()
                now = time.monotonic()
                if frame is None or (now - frame_t) > STALL_STOP_SEC:
                    self._stop()
                    if self._camera_wedged or (now - fresh_t) > CAPTURE_TIMEOUT + 2.0:
                        goal_handle.abort()
                        result.result_code = RC_ALIGN_FAILED
                        result.message = ('카메라 프레임 정지 — 도킹 중단'
                                          + ('(프리즈, 노드 재시작 필요)'
                                             if self._camera_wedged else ''))
                        log.error('[floor_dock] %s' % result.message)
                        return result
                    time.sleep(0.03)
                    continue
                fresh_t = now

                # 후진/회전/후퇴(블라인드) 중엔 정면 마커 무시(검출 건너뜀). VERIFY 는 검출 필요.
                found, d, bearing, yaw, plan, size = False, 0.0, 0.0, 0.0, None, 0.0
                blind = fsm.state in ('TURN', 'REVERSE', 'HOLD', 'ADVANCE')
                det = None if blind else detector.find_dock(frame, self._mapper)
                if det is not None:
                    center, heading, size, contour = det
                    d, bearing, yaw = fsm_mod.docking_values(center, heading)
                    th1, dist_pl, th2, _gx, _gy = fsm_mod.centerline_plan(center, heading)
                    plan, found = (th1, dist_pl, th2), True

                odom_yaw, odom_xy = self._odom_snapshot()
                v, w = fsm.update(found, d, bearing, yaw, odom_yaw, 0.0, 0.0,
                                  proceed=True, n=99 if found else 0,
                                  plan=plan, odom_xy=odom_xy)
                self._publish(v, w)

                if self._stream:
                    self._render_stream(frame, det, fsm, d, bearing, yaw, v, w)

                if self._debug and now - last_dbg >= 0.5:
                    oy = ('%+.1f' % math.degrees(odom_yaw)) if odom_yaw is not None else '--'
                    dw = (d + self._crossbar_to_wall) * 100
                    log.debug('[%s] dw=%5.1fcm b=%+5.1f y=%+5.1f odom=%s v=%+.3f w=%+.3f'
                              % (self._phase(fsm), dw if found else 0.0,
                                 math.degrees(bearing) if found else 0.0,
                                 math.degrees(yaw) if found else 0.0, oy, v, w)
                              if found else
                              '[%s] not found odom=%s v=%+.3f w=%+.3f'
                              % (self._phase(fsm), oy, v, w))
                    last_dbg = now

                if now - last_fb >= 0.2:
                    fb.phase = fsm.state
                    fb.marker_detected = found
                    fb.distance_to_wall_m = float(d + self._crossbar_to_wall) if found else 0.0
                    goal_handle.publish_feedback(fb)
                    last_fb = now

                if fsm.state in ('DONE', 'ABORT'):
                    self._stop()
                    self._fill_result(result, fsm)
                    if fsm.result_code == RC_OK:
                        goal_handle.succeed()
                        rv = ('  [REVERSE %s trav=%.1fcm]'
                              % ('ODOM' if fsm.reverse_odom_used else '⚠️TIME(폴백)',
                                 fsm.reverse_trav * 100)
                              ) if fsm.reverse_odom_used is not None else ''
                        log.info('[floor_dock] 완료: %s%s' % (result.message, rv))
                    else:
                        goal_handle.abort()
                        log.error('[floor_dock] 실패(%d): %s'
                                  % (fsm.result_code, result.message))
                    return result

                time.sleep(max(0.0, period - (time.monotonic() - t0)))

            self._stop()
            goal_handle.abort()
            result.result_code = RC_CANCELLED
            result.message = '노드 종료'
            return result
        finally:
            self._cam_release()

    @staticmethod
    def _phase(fsm):
        return ('CL:' + fsm.cl_phase) if fsm.state == 'CENTERLINE' else fsm.state

    @staticmethod
    def _fill_result(result, fsm):
        result.result_code = fsm.result_code
        result.message = fsm.note
        result.final_wall_gap_m = float(fsm.final_gap)
        b, y, d = fsm.last_bearing, fsm.last_yaw, fsm.last_d
        if b is None or y is None or d is None or abs(b) > 90 or abs(y) > 90:
            return
        sigma = fsm_mod._ang_norm(b - y)
        result.final_lateral_m = float(d * math.sin(sigma))
        result.final_yaw_error = float(y)

    # --- 웹 스트림(view only, 디버그) ---------------------------------- #
    def _render_stream(self, frame, det, fsm, d, bearing, yaw, v, w):
        vis = frame.copy()           # BGR
        c = (0, 255, 0) if det is not None else (0, 0, 255)
        if det is not None:
            center, heading, size, contour = det
            cv2.polylines(vis, [contour.astype(int)], True, (0, 255, 0), 2)
            dw = d + self._crossbar_to_wall
            _put(vis, 'dw=%.3fm(wall) b=%+.1f y=%+.1f' % (
                dw, math.degrees(bearing), math.degrees(yaw)), (10, 26), c)
        else:
            _put(vis, 'no H', (10, 26), c)
        _put(vis, '[%s] v=%+.3f w=%+.3f%s' % (self._phase(fsm), v, w,
             '  DRY-RUN' if self._dry_run else '  LIVE'), (10, 52), (255, 255, 0))
        ok, buf = cv2.imencode('.jpg', vis, self._stream_enc)
        if ok:
            with self._jpeg_lock:
                self._latest_jpeg = buf.tobytes()

    def _start_stream_server(self):
        server = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *a):
                pass

            def do_GET(self):
                if self.path == '/':
                    body = PAGE.encode()
                    self.send_response(200)
                    self.send_header('Content-Type', 'text/html; charset=utf-8')
                    self.send_header('Content-Length', str(len(body)))
                    self.end_headers()
                    self.wfile.write(body)
                elif self.path == '/stream':
                    self.send_response(200)
                    self.send_header('Content-Type',
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
                                    ('Content-Length: %d\r\n\r\n' % len(jpg)).encode())
                                self.wfile.write(jpg)
                                self.wfile.write(b'\r\n')
                            time.sleep(1.0 / STREAM_FPS)
                    except (BrokenPipeError, ConnectionResetError):
                        pass
                else:
                    self.send_error(404)

        self._httpd = ThreadingHTTPServer(('0.0.0.0', self._stream_port), Handler)
        threading.Thread(target=self._httpd.serve_forever, daemon=True).start()


def main(args=None):
    lock_fd, holder = acquire_single_instance()
    if lock_fd is None:
        print('floor_dock 서버가 이미 실행 중(PID %s) — 종료' % holder)
        return
    rclpy.init(args=args)
    node = FloorDockServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node._stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
