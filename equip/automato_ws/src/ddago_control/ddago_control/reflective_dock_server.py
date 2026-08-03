#!/usr/bin/env python3
"""반사테이프 라이다 후진 도킹 — ReflectiveDock Action 서버 (DCS → DdaGo).

충전소(E4 순찰 종료 복귀 / E2 22-1 작업 실패 복귀)의 반사테이프 코너 마커에
후진으로 접붙인다. floor_dock_server(바닥 H 마커, 카메라) 와 나란한 세 번째 도킹
서버이며, 구조·안전장치를 그대로 미러링한다. 다른 점은 입력 센서뿐:
  * FloorDock : 전면 카메라 → 바닥 H 마커 (homography)
  * ReflectiveDock : 라이다 → /docking_marker_pose (detector_node 가 발행)

검출은 이미 검증된 reflective_dock/detector_node.py(=`reflective_detector`)가
`/scan` 을 보고 `/docking_marker_pose` 로 발행한다. 이 서버는 그 pose + `/odom` 을
구독해 순수 FSM(reflective_fsm.ReflectiveDockFsm)을 실행 루프에서 굴리고,
`/cmd_vel` 로 후진 도킹을 수행한다. 로직 출처는 reflective_fsm 하나다
(dock_controller.py 디버그 노드도 같은 FSM 을 쓴다).

안전(floor_dock_server 미러):
  * 단일 인스턴스 파일 락(같은 액션 서버 둘 뜨는 사고 방지)
  * 종료·취소·실패 어느 경로로 빠져도 0 속도 반복(_stop). dry_run:=true 로 명령만 확인.
  * bringup 에 cmd_vel 워치독이 없으니 첫 투입은 반드시 dry_run 으로.

실패 재시도는 ACS 가 관장한다(N_dock 기본 3회).
"""
import fcntl
import math
import os
import threading
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.logging import LoggingSeverity
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty, Int64

from automato_interfaces.action import ReflectiveDock

from .reflective_dock import reflective_fsm as fsm_mod

LOCK_PATH = os.environ.get('DDAGO_REFLECTIVE_DOCK_LOCK',
                           '/tmp/ddago_reflective_dock_server.lock')


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


class ReflectiveDockServer(Node):
    def __init__(self, **kwargs):
        super().__init__('ddago_reflective_dock_server', **kwargs)
        self._cb = ReentrantCallbackGroup()

        self.declare_parameter('robot_id', 'dg_01')
        # 로봇 물리값(로봇별 config/reflective_dock/<robot>.yaml). 정지 목표 라이다거리
        # = rear_offset_m + stop_gap_m.
        self.declare_parameter('rear_offset_m', 0.10)
        self.declare_parameter('stop_gap_m', 0.02)
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('pose_topic', '/docking_marker_pose')
        self.declare_parameter('control_hz', 20.0)
        # 마커 pose 신선도 기준: 이보다 오래된 pose 는 '없음'으로 본다.
        self.declare_parameter('watchdog_sec', float(fsm_mod.WATCHDOG_SEC))
        self.declare_parameter('dry_run', False)
        self.declare_parameter('debug', False)
        # 도킹 시작 시 검출기의 목표 lock 을 버리게 할지. 끄면 옛 동작(검출기가 순찰
        # 중에 잡아 둔 lock 을 그대로 씀)으로 되돌아간다 — 문제 생겼을 때의 탈출구다.
        self.declare_parameter('reset_lock_on_start', True)
        # 리셋 알림 뒤 검출기가 새 프레임으로 다시 잡을 시간. SNAP 이 평균낼 프레임에
        # 옛 lock 기준 값이 섞이지 않게 한 박자 쉰다(라이다 10Hz → 3프레임분).
        self.declare_parameter('reset_lock_settle_sec', 0.3)
        # 속도 명령이 0 인 채 이만큼 지나면 지금 어느 단계인지 경고를 한 번 남긴다.
        # FSM 이 어떤 이유로 멈춰 있어도 로그로 드러나게 하는 안전망이다.
        self.declare_parameter('stall_warn_sec', 3.0)

        self._robot_id = self.get_parameter('robot_id').value
        self._dry_run = bool(self.get_parameter('dry_run').value)
        self._watchdog = float(self.get_parameter('watchdog_sec').value)
        self._period = 1.0 / max(float(self.get_parameter('control_hz').value), 1.0)
        if bool(self.get_parameter('debug').value):
            self.get_logger().set_level(LoggingSeverity.DEBUG)

        # 관측 스냅샷(콜백이 채우고 실행 루프가 읽는다).
        self._pose_lock = threading.Lock()
        self._pose = None          # (x, y, yaw) 마커 라이다프레임
        self._pose_t = 0.0
        self._odom_lock = threading.Lock()
        self._odom = None          # (X, Y, yaw) 로봇 odom
        self._busy = threading.Lock()

        pose_topic = self.get_parameter('pose_topic').value
        odom_topic = self.get_parameter('odom_topic').value
        self.create_subscription(PoseStamped, pose_topic, self._on_pose, 10,
                                 callback_group=self._cb)
        self.create_subscription(Odometry, odom_topic, self._on_odom, 10,
                                 callback_group=self._cb)
        self._cmd_pub = self.create_publisher(
            Twist, self.get_parameter('cmd_vel_topic').value, 10)
        # 현재 task 알림(telemetry_publisher 가 싣는다). floor/dock 서버와 동일 QoS.
        self._task_pub = self.create_publisher(
            Int64, '/ddago/current_task',
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        # 도킹 시작 알림 → detector_node 가 목표 lock 을 버린다(_run 첫머리 참고).
        self._lock_reset_pub = self.create_publisher(
            Empty, '/ddago/dock_lock_reset',
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))

        self._server = ActionServer(
            self, ReflectiveDock, '/ddago/reflective_dock',
            execute_callback=self._execute,
            cancel_callback=lambda _gh: CancelResponse.ACCEPT,
            callback_group=self._cb)

        self.get_logger().info(
            'ReflectiveDock 서버 준비됨: robot_id=%s → /ddago/reflective_dock, '
            'pose=%s odom=%s%s' % (
                self._robot_id, pose_topic, odom_topic,
                '  ⚠️DRY-RUN' if self._dry_run else ''))

    # --- 콜백: 최신 관측을 스냅샷에 저장 ------------------------------- #
    def _on_pose(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y
        # detector_node 는 순수-z 쿼터니언으로 yaw 를 싣는다 → yaw = 2·atan2(z, w).
        yaw = 2.0 * math.atan2(msg.pose.orientation.z, msg.pose.orientation.w)
        with self._pose_lock:
            self._pose = (x, y, yaw)
            self._pose_t = time.monotonic()

    def _on_odom(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = 2.0 * math.atan2(q.z, q.w)  # 평면 주행: 순수-z 로 충분(detector 와 동일)
        with self._odom_lock:
            self._odom = (p.x, p.y, yaw)

    def _pose_snapshot(self):
        with self._pose_lock:
            return self._pose, self._pose_t

    def _odom_snapshot(self):
        with self._odom_lock:
            return self._odom

    # --- cmd_vel ------------------------------------------------------- #
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
        result = ReflectiveDock.Result()
        if not self._busy.acquire(blocking=False):
            goal_handle.abort()
            result.result_code = fsm_mod.RC_ALIGN_FAILED
            result.message = '다른 도킹이 진행 중이다'
            return result
        try:
            return self._run(goal_handle, goal_handle.request, result)
        finally:
            self._stop()
            self._busy.release()

    def _run(self, goal_handle, goal, result):
        log = self.get_logger()
        rear = float(self.get_parameter('rear_offset_m').value)
        # goal.stop_gap_m > 0 이면 goal 이 우선, 아니면 노드 기본(config yaml).
        gap = (float(goal.stop_gap_m) if float(goal.stop_gap_m) > 0.0
               else float(self.get_parameter('stop_gap_m').value))
        fsm = fsm_mod.ReflectiveDockFsm(rear_offset_m=rear, stop_gap_m=gap)

        log.info('반사 도킹 시작: task=%d point=%s rear=%.3f gap=%.3f '
                 '(목표 라이다 %.1fcm)'
                 % (goal.task_id, goal.task_point_id, rear, gap, fsm.target_m * 100))
        task_msg = Int64()
        task_msg.data = int(goal.task_id)
        self._task_pub.publish(task_msg)

        # 찜해 둔 목표를 버리게 한다. 검출기는 순찰 내내 켜져 있어서 **주행 중에도**
        # lock 을 잡을 수 있는데(정지 게이트가 폴백으로 뚫리는 경우), 움직이며 본 마커는
        # 옆 충전소이거나 법선이 뒤집혀 있을 수 있다. 지금은 복귀 주행이 끝나 로봇이
        # 멈춰 있으므로, 여기서 버리고 다시 잡게 하면 게이트가 제대로 작동한다.
        # (2026-08-03 실사고: 도킹 7초 전, 초속 5cm 로 회전 중에 잡은 lock 으로 사전정렬이
        #  뒤집혀 로봇이 마커를 마주 본 채 후진 단계에 들어갔다.)
        if bool(self.get_parameter('reset_lock_on_start').value):
            self._lock_reset_pub.publish(Empty())
            settle = float(self.get_parameter('reset_lock_settle_sec').value)
            if settle > 0.0:
                time.sleep(settle)

        stall_warn = float(self.get_parameter('stall_warn_sec').value)
        stall_t0 = None          # 속도 0 이 시작된 시각
        stall_warned = False     # 같은 정지 구간에서 경고는 한 번만
        fb = ReflectiveDock.Feedback()
        last_fb = 0.0
        prev_state = None
        while rclpy.ok():
            t0 = time.monotonic()
            if goal_handle.is_cancel_requested:
                self._stop()
                goal_handle.canceled()
                result.result_code = fsm_mod.RC_CANCELLED
                result.message = '취소됨'
                log.warn('[reflective_dock] 취소 — 정지')
                return result

            now = time.monotonic()
            pose, pose_t = self._pose_snapshot()
            fresh = pose is not None and (now - pose_t) <= self._watchdog
            odom = self._odom_snapshot()

            v, w = fsm.step(pose if fresh else None, odom, now)
            self._publish(v, w)

            if fsm.state != prev_state:      # 상태 전이만 로깅(순수 FSM 은 로그 안 함)
                extra = ('  (법선까지 %.1fcm)' % (fsm.snap_move * 100)
                         if fsm.state in ('TURN1', 'TURN2') and fsm.snap_move else '')
                log.info('[reflective_dock] → %s%s' % (fsm.state, extra))
                prev_state = fsm.state
                stall_t0, stall_warned = None, False   # 단계가 바뀌면 정지 감시 초기화

            # 정지 감시 — 바퀴가 안 도는 채로 시간만 가는 상황을 로그로 드러낸다.
            # 상태 전이가 없으면 위 로그도 안 나오므로, 이게 없으면 '아무 일도 안 일어나는
            # 것처럼 보이는 구간'이 통째로 깜깜해진다(실사고 때 48초가 그랬다).
            if abs(v) < 1e-6 and abs(w) < 1e-6:
                if stall_t0 is None:
                    stall_t0 = now
                elif not stall_warned and (now - stall_t0) >= stall_warn:
                    log.warn('[reflective_dock] %s 에서 %.1f초째 정지 명령만 나간다 '
                             '(마커=%s 거리=%s) — 진행이 막혔는지 확인'
                             % (fsm.state, now - stall_t0,
                                '보임' if fsm.marker_detected else '없음',
                                '%.2fm' % fsm.last_d if fsm.last_d else '-'))
                    stall_warned = True
            else:
                stall_t0, stall_warned = None, False

            if now - last_fb >= 0.2:
                fb.phase = fsm.state
                fb.marker_detected = bool(fsm.marker_detected)
                fb.distance_to_marker_m = float(fsm.last_d) if fsm.last_d else 0.0
                goal_handle.publish_feedback(fb)
                last_fb = now

            if fsm.done:
                self._stop()
                result.result_code = int(fsm.result_code)
                result.message = fsm.note
                result.final_gap_m = float(fsm.final_gap)
                result.final_lateral_m = float(fsm.final_lateral)
                result.final_yaw_error = float(fsm.final_yaw_error)
                if fsm.result_code == fsm_mod.RC_OK:
                    goal_handle.succeed()
                    log.info('[reflective_dock] 완료: %s (gap=%.1fcm e=%.1fcm tilt=%.1f°)'
                             % (fsm.note, fsm.final_gap * 100, fsm.final_lateral * 100,
                                math.degrees(fsm.final_yaw_error)))
                else:
                    goal_handle.abort()
                    log.error('[reflective_dock] 실패(%d): %s'
                              % (fsm.result_code, fsm.note))
                return result

            time.sleep(max(0.0, self._period - (time.monotonic() - t0)))

        self._stop()
        goal_handle.abort()
        result.result_code = fsm_mod.RC_CANCELLED
        result.message = '노드 종료'
        return result


def main(args=None):
    lock_fd, holder = acquire_single_instance()
    if lock_fd is None:
        print('reflective_dock 서버가 이미 실행 중(PID %s) — 종료' % holder)
        return
    rclpy.init(args=args)
    node = ReflectiveDockServer()
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
