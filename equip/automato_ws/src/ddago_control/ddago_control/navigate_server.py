#!/usr/bin/env python3
"""RP-113  E1/E2: DdaGo(주행 로봇) Navigate Action 서버 — 경로 배열 순차 주행.

ACS가 **예약을 확보한 구간까지**를 Waypoint 배열로 한 번에 하달하면(E1 6, E2 20),
로봇은 그 배열을 앞에서부터 순서대로 Nav2 로 주행한다. 로봇은 예약도 순찰 지점도
모르고, 받은 배열을 소화할 뿐이다.

이 노드가 맡는 역할:
  * Navigate 액션의 **서버**              (DCS → DdaGo, /ddago/navigate)
  * Nav2 NavigateToPose 액션의 **클라이언트**  (DdaGo → Nav2)
  * cmd_vel 의 **발행자**                     (정밀 조준·출발 정렬의 미세 동작만)
  * AnalyzeFrame 서비스의 **클라이언트**     (DdaGo → DCS, /dg/analyze_frame)
  * CaptureFrame 서비스의 **클라이언트**     (DdaGo → 카메라 노드, /ddago/capture_frame)
  * 현재 task 알림의 **발행자**             (DdaGo 내부, /ddago/current_task)

액션 이름을 절대이름 `/ddago/navigate` 로 두는 이유:
  로봇 구성이 물리적으로 분리되어(로봇 1대 = 1망) 로봇 쪽 이름에는 robot_id
  네임스페이스를 붙이지 않기로 했다. 텔레메트리(/ddago/telemetry)와 같은 규칙이다.
  절대이름이라 launch 에서 PushRosNamespace 를 써도 이 액션만은 영향받지 않는다.

**피드백은 노드에 '도착한 순간'에만 발행한다.** 주행 중에는 보내지 않는다.
ACS 는 Feedback.current_waypoint_id 를 "로봇이 그 노드까지 갔다"로 읽고 거기까지
오는 데 쓴 통로 예약을 반납하기 때문이다(patrol_dispatcher._passed_resources).
아직 가는 중인 목표 노드를 미리 실어 보내면 ACS 가 로봇이 지금 지나고 있는 통로를
남에게 내주게 되어 정면 충돌로 이어진다.

동시성: execute 콜백이 Nav2 결과를 기다리며 블로킹하는 동안에도 Nav2 피드백 콜백이
계속 돌아야 하므로 MultiThreadedExecutor + ReentrantCallbackGroup 을 쓴다(main 참고).
Nav2 결과 대기는 patrol_server 에서 검증된 _spin_wait 폴링 패턴을 그대로 채택한다.

**Nav2 에 넘기는 도착 방향은 목표 yaw 가 아니라 '그리로 가는 이동 방향'이다.**
목표 yaw 를 그대로 주면 Nav2 의 컨트롤러(RPP)가 도착해서 제자리 회전을 하는데, 그
회전이 등속이라 목표각을 지나쳤다 되돌아오길 반복한다(두리번거림). 대신 이 노드가
직접 세 가지를 한다 — 셋 다 남은 양에 비례해 감속하고, 지나치면 되돌리지 않고 멈춘다.
  ① 출발 정렬 : Nav2 에 목표를 주기 **전에** 이동 방향으로 미리 돈다.
                RPP 는 출발할 때도 20°(rotate_to_heading_min_angle) 넘게 벌어지면
                제자리 회전을 하는데 그것도 등속이라 헌팅한다. 미리 돌려 두면 안 돈다.
  ② 이동 방향 : Nav2 goal 의 yaw = 현재 위치에서 목표로 가는 방향.
  ③ 정밀 조준 : 촬영 지점에 도착하면 목표 좌표·yaw 로 좁힌 뒤 찍는다.
                (실측: 도착 fwd +4.3cm / yaw ±29° → 보정 후 ±0.5cm / ±1.1°)
계산은 precision_move 모듈(순수 함수)에 있고, 여기서는 위치 조회·cmd_vel 발행·로그만
맡는다. TF(map/odom → base)를 못 받으면 세 동작을 모두 건너뛰고 경고만 남긴다 —
정밀 조준은 사진 품질 문제이지 주행 성공 조건이 아니기 때문이다.

**제자리 180° 회전(짝-스핀)은 하지 않는다.** 예전에는 '직전 원소와 좌표가 같으면
Nav2 Spin' 으로 분기했으나(E2 20-1), 현장 측정 결과 그 회전이 **물리적으로 불가능**
했다: 로봇(12cm 정사각)이 45° 기울면 옆으로 8.49cm 튀어나오는데 통로 여유가 7.5cm 다.
같은 자리 반대 방향 촬영은 ACS 가 왕복 경로로 두 번 지나가게 해서 해결한다.

⚠️ bringup 에 cmd_vel 워치독이 없다 — 마지막 속도 명령이 계속 유지된다. 미세 동작은
성공·실패·타임아웃 어느 경로로 빠지든 반드시 정지 명령을 거치게 되어 있다(_halt).

**촬영은 capture == true 인 노드에서만 한다**(E2 3단계). 나머지 노드는 통과만 한다.
어디서 찍을지는 ACS 가 정해서 플래그로 알려주며(`capture = 순찰지점 AND 미방문`),
로봇은 그 플래그만 본다. 분석 요청은 응답을 기다리지 않고 던져(fire-and-forget)
다음 waypoint 주행을 막지 않는다.

파라미터:
  robot_id               (str)   로그 표기용 로봇 식별자          기본 'dg_01'
  nav2_action            (str)   Nav2 주행 액션 이름(상대)        기본 'navigate_to_pose'
  capture_service        (str)   카메라 노드 촬영 서비스(절대)     기본 '/ddago/capture_frame'
  analyze_service        (str)   DCS 분석 서비스(절대)            기본 '/dg/analyze_frame'
                                 (로봇 공용이라 네임스페이스 안 붙임 → 절대이름)
  arrival_settle_sec     (float) 도착 후 잔상 방지 정지 대기       기본 0.3
  capture_timeout_sec    (float) CaptureFrame 응답 대기 상한       기본 10.0
                                 (카메라 노드가 배터리 절약으로 유휴 시 웹캠을
                                  놓기 때문에, 콜드 촬영은 재오픈+AE 워밍업 포함
                                  실측 4~5초가 걸린다. 5초는 상시 열림 시절 기준.
                                  웹캠이 아예 없으면 노드가 즉시 실패 응답하므로
                                  10초를 실제로 기다리는 건 비정상 상황뿐이다)
  nav2_wait_sec          (float) Nav2 서버/goal 수락 대기          기본 10.0
  nav2_result_timeout_sec(float) waypoint 1개당 결과 대기 상한     기본 300.0

  --- 정밀 주행 ---
  precision_enable       (bool)  출발정렬·이동방향·정밀조준 사용    기본 True
                                 False 면 예전처럼 목표 yaw 를 Nav2 에 그대로 넘긴다
  global_frame_id        (str)   목표 좌표와 비교할 좌표계          기본 'map'
  odom_frame_id          (str)   이동량 측정용 좌표계               기본 'odom'
  base_frame_id          (str)   로봇 본체 프레임                  기본 'base_footprint'
  cmd_vel_topic          (str)   미세 동작 주행 명령 토픽(상대)     기본 'cmd_vel'
  refine_rounds          (int)   회전→전후진 반복 횟수              기본 2
  refine_step_timeout_sec(float) 미세 동작 1회 제한 시간            기본 5.0
  refine_tol_lin_m       (float) 여기 들어오면 됐다고 본다          기본 0.005
  refine_tol_ang_deg     (float) 위와 같음(각도)                   기본 1.0
  refine_max_fix_lin_m   (float) 이보다 크게 어긋나면 손대지 않음   기본 0.12
  min_travel_m           (float) 이보다 짧으면 '가는 방향' 계산 생략 기본 0.10
"""
import math
import threading
import time

from action_msgs.msg import GoalStatus
from automato_interfaces.action import Navigate
from automato_interfaces.srv import AnalyzeFrame, CaptureFrame
from ddago_control import precision_move as pm
from geometry_msgs.msg import Twist
import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Int64
from tf2_ros import Buffer, TransformListener

# Result.last_waypoint_id: 배열의 첫 노드에도 도달하지 못했음을 뜻하는 값.
# ACS 는 세그먼트 경로에 없는 값을 받으면 "세그먼트에 진입조차 못 했다"로 보고
# 진입 지점 기준으로 재계획한다(_segment_progress). wps[0] 을 보내면 "첫 노드까진
# 갔다"고 오해해 엉뚱한 통로를 막힘으로 판정하므로 반드시 이 값을 쓴다.
NO_WAYPOINT_REACHED = -1


def _yaw_from_quaternion(q):
    """쿼터니언 메시지 → yaw(rad). 계산 자체는 precision_move 가 한다."""
    return pm.yaw_from_quaternion(q.x, q.y, q.z, q.w)


def _fmt_err(e):
    """(fwd, lat, dyaw) 오차를 사람이 읽을 한 줄로.

    fwd 는 화면이 가로로 밀리는 양, lat 은 피사체까지의 거리, dyaw 는 각도 오차다.
    현장에서 리포트를 이 형식으로 읽어 왔으므로 같은 표기를 유지한다.
    """
    if e is None:
        return '측정불가'
    return (f'fwd={e[0] * 100:+5.1f}cm  lat={e[1] * 100:+5.1f}cm  '
            f'yaw={math.degrees(e[2]):+5.1f}°')


def _spin_wait(future, timeout, poll=0.5, on_wait=None):
    """백그라운드 executor 가 완료할 future 를 (다른 스레드에서) 기다린다.

    MultiThreadedExecutor 로 노드를 spin 하면 콜백은 executor 스레드에서 돌고,
    이 함수는 execute 콜백 스레드에서 future 를 폴링한다. add_done_callback 으로
    완료 이벤트를 받되 poll 간격마다 깨어나 on_wait(예: 취소 확인)을 실행한다.
    타임아웃/예외 시 None 반환.
    """
    done = threading.Event()
    future.add_done_callback(lambda _f: done.set())
    waited = 0.0
    while not done.wait(poll):
        if on_wait is not None:
            on_wait()
        waited += poll
        if timeout is not None and waited >= timeout:
            return None
    try:
        return future.result()
    except Exception:  # noqa: BLE001
        return None


class NavigateServer(Node):
    def __init__(self, **kwargs):
        # **kwargs 는 테스트에서 parameter_overrides 등을 주입하기 위한 통로.
        super().__init__('navigate_server', **kwargs)

        # --- 파라미터 ---
        self.declare_parameter('robot_id', 'dg_01')
        self.declare_parameter('nav2_action', 'navigate_to_pose')
        self.declare_parameter('capture_service', '/ddago/capture_frame')
        self.declare_parameter('analyze_service', '/dg/analyze_frame')
        self.declare_parameter('arrival_settle_sec', 0.3)
        self.declare_parameter('nav2_wait_sec', 10.0)
        self.declare_parameter('nav2_result_timeout_sec', 300.0)
        self.declare_parameter('capture_timeout_sec', 10.0)

        self._robot_id = self.get_parameter('robot_id').value
        self._nav2_action = self.get_parameter('nav2_action').value
        capture_service = self.get_parameter('capture_service').value
        analyze_service = self.get_parameter('analyze_service').value
        self._settle_sec = float(self.get_parameter('arrival_settle_sec').value)
        self._nav2_wait = float(self.get_parameter('nav2_wait_sec').value)
        self._nav2_result_timeout = float(
            self.get_parameter('nav2_result_timeout_sec').value)
        self._capture_timeout = float(
            self.get_parameter('capture_timeout_sec').value)

        # --- 정밀 주행 파라미터 ---
        self.declare_parameter('precision_enable', True)
        self.declare_parameter('global_frame_id', 'map')
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'base_footprint')
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
        self.declare_parameter('refine_rounds', 2)
        self.declare_parameter('refine_step_timeout_sec', 5.0)
        self.declare_parameter('refine_tol_lin_m', pm.TOL_LIN)
        self.declare_parameter('refine_tol_ang_deg', 1.0)
        self.declare_parameter('refine_max_fix_lin_m', pm.MAX_FIX_LIN)
        self.declare_parameter('min_travel_m', pm.MIN_TRAVEL)

        self._precision = bool(self.get_parameter('precision_enable').value)
        self._global_frame = self.get_parameter('global_frame_id').value
        self._odom_frame = self.get_parameter('odom_frame_id').value
        self._base_frame = self.get_parameter('base_frame_id').value
        self._refine_rounds = int(self.get_parameter('refine_rounds').value)
        self._refine_timeout = float(
            self.get_parameter('refine_step_timeout_sec').value)
        self._tol_lin = float(self.get_parameter('refine_tol_lin_m').value)
        self._tol_ang = math.radians(
            float(self.get_parameter('refine_tol_ang_deg').value))
        self._max_fix_lin = float(
            self.get_parameter('refine_max_fix_lin_m').value)
        self._min_travel = float(self.get_parameter('min_travel_m').value)

        # 서버 실행(블로킹) 중에도 Nav2 콜백이 처리되도록 모든 통신을
        # ReentrantCallbackGroup 에 넣는다(main 의 MultiThreadedExecutor 와 짝).
        self._cb = ReentrantCallbackGroup()

        # --- 상태 ---
        self._nav_client = None     # Nav2 NavigateToPose 클라이언트 (지연 생성)
        self._last_pose = None      # Nav2 가 보고한 최신 (x, y, yaw). TF 실패 시 폴백
        self._pending_analyze = []  # call_async future 보관(GC 방지)

        # --- 정밀 주행: 위치 조회(TF) + 미세 주행 명령(cmd_vel) ---
        # Nav2 는 '달리면서' 맞추는 단계라 여기 관여하지 않는다. Nav2 goal 이 끝난 뒤
        # (또는 보내기 전) 로봇이 서 있을 때만 이 발행자로 직접 바퀴를 굴린다.
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._vel_pub = self.create_publisher(
            Twist, self.get_parameter('cmd_vel_topic').value, 10)

        # --- 촬영 서비스 클라이언트 (DdaGo → 카메라 노드) ---
        # capture 지점에 도착하면 이 클라이언트로 프레임 1장을 요청한다(토픽 구독 대신).
        self._capture_client = self.create_client(
            CaptureFrame, capture_service, callback_group=self._cb)

        # --- 분석요청 서비스 클라이언트 ---
        self._analyze_client = self.create_client(
            AnalyzeFrame, analyze_service, callback_group=self._cb)

        # --- 현재 task 알림 (로봇 내부 신호) ---
        # 텔레메트리의 task_id 를 채우려면 telemetry_publisher 가 "지금 어느 task 를
        # 수행 중인지" 알아야 하는데, 두 노드는 프로세스가 달라 변수를 공유할 수 없다.
        # latched(TRANSIENT_LOCAL, depth 1) 라 구독자가 나중에 떠도 마지막 값을 받는다.
        self._task_pub = self.create_publisher(
            Int64, '/ddago/current_task',
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )

        # --- Navigate 액션 서버 (절대이름 → 네임스페이스 영향 없음) ---
        self._server = ActionServer(
            self, Navigate, '/ddago/navigate',
            execute_callback=self._execute,
            cancel_callback=lambda _gh: CancelResponse.ACCEPT,
            callback_group=self._cb,
        )

        self.get_logger().info(
            f'Navigate 서버 준비됨: robot_id={self._robot_id} → /ddago/navigate, '
            f'Nav2={self._nav2_action}, '
            f'촬영={capture_service}, 분석={analyze_service}, '
            f'정밀주행={"ON" if self._precision else "OFF"}'
            f'({self._global_frame}/{self._odom_frame}→{self._base_frame})')

    # ------------------------------------------------------------------ #
    # 현재 task 알림: goal 을 받을 때마다 1회 발행 (latched 라 재발행 불필요)
    # ------------------------------------------------------------------ #
    def _publish_current_task(self, task_id):
        msg = Int64()
        msg.data = int(task_id)
        self._task_pub.publish(msg)
        self.get_logger().info(
            f'현재 task 알림 → /ddago/current_task: task_id={msg.data}')

    # ------------------------------------------------------------------ #
    # Navigate 실행 콜백: 배열을 앞에서부터 순서대로 주행
    # ------------------------------------------------------------------ #
    def _execute(self, goal_handle):
        req = goal_handle.request
        wps = list(req.waypoints)
        task_id = req.task_id

        if not wps:
            # 빈 배열은 ACS 의 버그다. 주행할 것이 없으니 즉시 실패로 알린다.
            self.get_logger().error(
                f'Navigate 수신 task={task_id} 이나 waypoints 가 비어 있음 → 실패(1)')
            goal_handle.abort()
            return self._make_result(1, NO_WAYPOINT_REACHED, 'empty waypoints')

        self.get_logger().info(
            f'Navigate 수신 task={task_id} '
            f'waypoints={[int(w.waypoint_id) for w in wps]} → 구간 주행 시작')

        # 이 goal 의 task_id 를 로봇 안에 알린다 → telemetry 가 task_id 를 싣는다.
        # goal 이 끝나도 0 으로 되돌리지 않는다: ACS 는 한 task 를 예약 구간 단위로
        # 쪼개 여러 goal 로 하달하므로(E2 4단계), goal 사이의 틈마다 0 이 되면
        # QT 화면에서 task_id 가 깜빡이고 22-1·E4 의 복귀 추적도 끊긴다.
        self._publish_current_task(task_id)

        last_wp = NO_WAYPOINT_REACHED
        code = 0
        for idx, wp in enumerate(wps):
            # 다음 waypoint 로 출발하기 전에 취소를 확인한다. 주행 중 취소는
            # _drive_to 안에서 Nav2 goal 취소로 이어진다.
            if goal_handle.is_cancel_requested:
                self.get_logger().warn(
                    f'취소 요청 확인 task={task_id} → 구간 중단 '
                    f'(마지막 도달 노드={last_wp})')
                code = 2
                break

            self.get_logger().info(
                f'주행 시작 task={task_id} [{idx + 1}/{len(wps)}] '
                f'waypoint={int(wp.waypoint_id)} '
                f'({wp.x:.2f},{wp.y:.2f}) yaw={wp.yaw:.2f} '
                f'{"촬영" if wp.capture else "통과"}')
            code = self._drive_to(goal_handle, wp)
            if code != 0:
                self.get_logger().warn(
                    f'주행 실패·중단 task={task_id} '
                    f'waypoint={int(wp.waypoint_id)} code={code} → 구간 종료 '
                    f'(마지막 도달 노드={last_wp})')
                break

            # 촬영 지점만 정밀 조준한다. 통과 지점은 다음 waypoint 로 출발할 때
            # 어차피 다시 도므로(_drive_to 의 출발 정렬), 여기서 맞춰 봐야 회전이
            # 두 번이 된다. 현장에서 스텝당 회전 2회 → 1회로 줄인 것이 이 규칙이다.
            #
            # 도착 보고보다 **먼저** 조준한다. ACS 는 도착 보고를 받으면 지나온 통로
            # 예약을 반납하는데(patrol_dispatcher._passed_resources), 조준은 최대
            # 12cm 뒤로 물러날 수 있어 방금 반납한 통로로 되돌아갈 여지가 있다.
            # 늦어지는 것은 2~4초뿐이다.
            if wp.capture:
                self._refine(wp)

            # 여기서부터 "도착 확정". 이 순서를 지켜야 last_wp 와 피드백이 어긋나지 않는다.
            last_wp = int(wp.waypoint_id)
            self._publish_arrival(goal_handle, wp, idx)

            # 촬영은 도착 보고 **뒤에** 한다. ACS 는 도착 보고를 기점으로 통로 반납과
            # 다음 구간 선예약을 시작하는데(E2 2단계), 촬영을 먼저 하면 그만큼 늦어진다.
            if wp.capture:
                self._capture_and_request(task_id, int(wp.waypoint_id))
            else:
                self.get_logger().debug(
                    f'waypoint={int(wp.waypoint_id)} 통과(capture=false)')

        result = self._make_result(code, last_wp, {
            0: 'arrived',
            1: 'failed/blocked',
            2: 'canceled',
        }.get(code, 'unknown'))

        if code == 0:
            goal_handle.succeed()
        elif code == 2 and goal_handle.is_cancel_requested:
            # canceled() 는 취소가 실제로 요청된 goal 에서만 호출할 수 있다.
            # Nav2 가 자체 사정으로 CANCELED 를 돌려준 경우는 abort 로 떨어뜨린다.
            goal_handle.canceled()
        else:
            goal_handle.abort()

        self.get_logger().info(
            f'Navigate 종료 task={task_id} result_code={code} '
            f'last_waypoint_id={last_wp}')
        return result

    @staticmethod
    def _make_result(code, last_wp, message):
        result = Navigate.Result()
        result.result_code = int(code)
        result.last_waypoint_id = int(last_wp)
        result.message = message
        return result

    # ------------------------------------------------------------------ #
    # 도착 보고: 노드에 실제로 도착한 순간에만 1회 발행 (E2 1단계)
    # ------------------------------------------------------------------ #
    def _publish_arrival(self, goal_handle, wp, idx):
        fb = Navigate.Feedback()
        fb.current_waypoint_id = int(wp.waypoint_id)
        fb.waypoint_index = int(idx)      # 이번 배열에서 몇 번째인지 (0-based)
        # 좌표는 TF(map→base) 실측을 1순위로 쓴다. Nav2 피드백(_last_pose)은 주행
        # **중**의 마지막 값이라 정밀 조준으로 움직인 만큼이 빠져 있다. TF 도 못 읽으면
        # 목표 좌표로 대신한다(도착했으므로 목표와 거의 같다).
        p = self._pose(self._global_frame)
        if p is not None:
            x, y, yaw = p
        elif self._last_pose is not None:
            x, y, yaw = self._last_pose
        else:
            x, y, yaw = wp.x, wp.y, wp.yaw
        fb.current_x = float(x)
        fb.current_y = float(y)
        fb.current_yaw = float(yaw)
        goal_handle.publish_feedback(fb)
        self.get_logger().info(
            f'도착 보고 waypoint={fb.current_waypoint_id} index={fb.waypoint_index} '
            f'({fb.current_x:.2f},{fb.current_y:.2f},{fb.current_yaw:.2f})')

    # ------------------------------------------------------------------ #
    # 촬영: settle 대기 → CaptureFrame 호출 → 받은 프레임을 analyze_frame 으로
    # ------------------------------------------------------------------ #
    def _capture_and_request(self, task_id, waypoint_id):
        # 정지 잔상 방지용 짧은 대기. 로봇은 도착해 멈춰 있고, 이 촬영이 끝나기
        # 전에는 다음 waypoint 로 출발하지 않으므로 '정지 순간의 프레임'이 보장된다.
        time.sleep(self._settle_sec)

        if not self._capture_client.service_is_ready():
            self.get_logger().warn(
                f'waypoint={waypoint_id} 촬영 대상이나 카메라 서비스 미준비 → '
                f'분석요청 스킵(주행은 계속). 카메라 노드 확인 필요')
            return

        request = CaptureFrame.Request()
        request.task_id = int(task_id)
        request.waypoint_id = int(waypoint_id)
        # 응답(프레임)을 받아야 분석으로 넘길 수 있으므로 여기서는 기다린다.
        # Nav2 결과 대기와 같은 _spin_wait 폴링을 재사용한다(대기 중 다른 콜백 진행).
        resp = _spin_wait(
            self._capture_client.call_async(request), self._capture_timeout)
        if resp is None:
            self.get_logger().warn(
                f'waypoint={waypoint_id} CaptureFrame 응답 타임아웃 → 분석요청 스킵')
            return
        if not resp.success:
            self.get_logger().warn(
                f'waypoint={waypoint_id} 촬영 실패({resp.message}) → 분석요청 스킵')
            return
        self._request_analyze(task_id, waypoint_id, resp.image)

    def _request_analyze(self, task_id, waypoint_id, frame):
        """캡처한 원본 프레임을 /dg/analyze_frame 에 던진다(응답 안 기다림)."""
        if not self._analyze_client.service_is_ready():
            self.get_logger().warn(
                'analyze_frame 서비스 미준비 → 이번 프레임 분석요청 스킵 '
                '(DCS 분석 서버 확인 필요)')
            return

        request = AnalyzeFrame.Request()
        request.task_id = int(task_id)
        request.waypoint_id = int(waypoint_id)
        request.image = frame   # 원본 그대로 전달 (jpeg/base64 변환은 DCS 몫)

        future = self._analyze_client.call_async(request)
        self._pending_analyze.append(future)   # GC 방지
        future.add_done_callback(
            lambda f: self._on_analyze_done(f, waypoint_id))
        self.get_logger().info(
            f'analyze_frame 요청 전송 task={task_id} waypoint={waypoint_id}')

    def _on_analyze_done(self, future, waypoint_id):
        if future in self._pending_analyze:
            self._pending_analyze.remove(future)
        try:
            resp = future.result()
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(
                f'analyze_frame 응답 실패 waypoint={waypoint_id}: {e}')
            return
        if resp.accepted:
            self.get_logger().info(
                f'analyze_frame 수락됨 waypoint={waypoint_id} '
                f'request_id={resp.request_id}')
        else:
            self.get_logger().warn(
                f'analyze_frame 거부됨 waypoint={waypoint_id}')

    # ------------------------------------------------------------------ #
    # 주행: Nav2 NavigateToPose 로 waypoint 1개까지. result_code(0/1/2) 반환
    # ------------------------------------------------------------------ #
    def _drive_to(self, goal_handle, wp):
        # 지연 임포트: nav2_msgs 없는 개발환경에서도 이 모듈이 임포트되게.
        from geometry_msgs.msg import PoseStamped
        from nav2_msgs.action import NavigateToPose
        from rclpy.action import ActionClient

        if self._nav_client is None:
            self._nav_client = ActionClient(
                self, NavigateToPose, self._nav2_action,
                callback_group=self._cb)

        if not self._nav_client.wait_for_server(timeout_sec=self._nav2_wait):
            self.get_logger().warn(
                f'Nav2 {self._nav2_action} 서버 없음 → 실패(1)')
            return 1

        # Nav2 에 넘길 도착 방향. 기본은 목표 yaw 지만, 정밀 주행이 가능하면
        # '그리로 가는 방향'으로 바꾸고 출발 전에 그 방향으로 미리 돌아 둔다.
        # 이렇게 하면 Nav2(RPP)는 출발할 때도 도착할 때도 제자리 회전을 하지 않는다.
        # 목표 방향 맞추기는 도착 후 _refine 이 감속하며 대신한다.
        nav_yaw = float(wp.yaw)
        cur = self._pose(self._global_frame) if self._precision else None
        if cur is not None:
            heading = pm.travel_yaw(
                (cur[0], cur[1]), (float(wp.x), float(wp.y)), self._min_travel)
            if heading is None:
                # 너무 짧은 이동 — atan2 가 위치 노이즈를 그대로 방향으로 바꾼다.
                self.get_logger().debug(
                    f'waypoint={int(wp.waypoint_id)} 이동거리 < '
                    f'{self._min_travel:.2f}m → 목표 yaw 를 그대로 사용')
            else:
                nav_yaw = heading
                self._align_to(nav_yaw, int(wp.waypoint_id))
        elif self._precision:
            self.get_logger().warn(
                f'{self._global_frame}→{self._base_frame} TF 없음 → 출발정렬·'
                f'이동방향 생략(주행은 계속). Nav2(AMCL) 기동 여부 확인 필요')

        nav_goal = NavigateToPose.Goal()
        ps = PoseStamped()
        ps.header.frame_id = 'map'
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = float(wp.x)
        ps.pose.position.y = float(wp.y)
        ps.pose.orientation.z, ps.pose.orientation.w = pm.quaternion_from_yaw(
            nav_yaw)
        nav_goal.pose = ps

        # Nav2 피드백은 ACS 로 중계하지 않고 현재 위치 캐시만 갱신한다.
        # 도착 보고(_publish_arrival)에서 이 값을 실어 보낸다.
        def _fb(nav_fb):
            p = nav_fb.feedback.current_pose.pose
            self._last_pose = (
                p.position.x, p.position.y, _yaw_from_quaternion(p.orientation))

        nav_handle = _spin_wait(
            self._nav_client.send_goal_async(nav_goal, feedback_callback=_fb),
            self._nav2_wait)
        if nav_handle is None or not nav_handle.accepted:
            self.get_logger().warn('Nav2 goal 거부/타임아웃 → 실패(1)')
            return 1

        # 결과 대기. 상위(ACS)가 취소 요청하면 Nav2 goal 도 취소한다(E2 22-1).
        def _check_cancel():
            if goal_handle.is_cancel_requested:
                nav_handle.cancel_goal_async()

        result_resp = _spin_wait(
            nav_handle.get_result_async(), self._nav2_result_timeout,
            on_wait=_check_cancel)
        if result_resp is None:
            self.get_logger().warn('Nav2 결과 타임아웃 → 실패(1)')
            return 1

        status = result_resp.status
        if status == GoalStatus.STATUS_SUCCEEDED:
            return 0
        if status == GoalStatus.STATUS_CANCELED:
            return 2
        return 1   # ABORTED 등

    # ------------------------------------------------------------------ #
    # 정밀 주행 ① 위치 조회 — 모든 미세 동작의 기준
    # ------------------------------------------------------------------ #
    def _pose(self, parent):
        """parent 좌표계 기준 로봇의 (x, y, yaw). 못 읽으면 None.

        map  : 목표 좌표와 비교할 때(절대 위치).
        odom : 몇 cm·몇 도를 움직였는지 잴 때. map 은 AMCL 이 보정할 때 순간 이동
               (점프)하는 일이 있어, 짧은 이동량 측정에는 연속적인 odom 이 정확하다.

        TF 를 못 읽는 것은 흔한 정상 상황이다(Nav2 미기동, 부팅 직후 캐시 비어 있음).
        예외를 밖으로 던지지 않고 None 으로 알려, 호출부가 정밀 동작만 건너뛰게 한다.
        """
        try:
            tr = self._tf_buffer.lookup_transform(
                parent, self._base_frame, rclpy.time.Time())
        except Exception:  # noqa: BLE001  (tf 가 아직 안 찼을 때도 여기로 온다)
            return None
        t = tr.transform.translation
        return t.x, t.y, _yaw_from_quaternion(tr.transform.rotation)

    def _error(self, target):
        """map 기준 목표 대비 (전방, 횡, 각) 오차. 위치를 못 읽으면 None."""
        p = self._pose(self._global_frame)
        return None if p is None else pm.pose_error(p, target)

    # ------------------------------------------------------------------ #
    # 정밀 주행 ② 미세 동작 — cmd_vel 직접 발행 (Nav2 goal 이 없는 동안에만)
    # ------------------------------------------------------------------ #
    def _send_vel(self, vx, wz):
        msg = Twist()
        msg.linear.x = float(vx)
        msg.angular.z = float(wz)
        self._vel_pub.publish(msg)

    def _halt(self):
        """정지 명령을 반복 발행한다.

        bringup 에 cmd_vel 워치독이 없어 **마지막 속도 명령이 그대로 유지**된다.
        미세 동작이 어떤 이유로 끝나든(도달·타임아웃·TF 유실) 반드시 여기를 지나야
        로봇이 멈춘다. 한 번은 유실될 수 있어 세 번 보낸다.
        """
        for _ in range(3):
            self._send_vel(0.0, 0.0)
            time.sleep(0.02)

    def _nudge_lin(self, dist):
        """dist[m] 만큼 전(+)/후(-)진. 이동량은 odom 으로 재며 감속한다.

        rclpy.spin_once 를 부르지 않는 것에 주의 — 이 노드는 MultiThreadedExecutor 가
        다른 스레드에서 계속 spin 하고 있어, 여기서 또 spin 하면 콜백이 두 스레드에서
        겹쳐 돈다. 여기서는 자고 일어나 TF 캐시를 읽기만 하면 된다.
        """
        start = self._pose(self._odom_frame)
        if start is None:
            return
        sign = 1.0 if dist > 0 else -1.0
        goal = abs(dist)
        deadline = time.monotonic() + self._refine_timeout
        try:
            while time.monotonic() < deadline:
                cur = self._pose(self._odom_frame)
                if cur is None:
                    break
                remain = goal - math.hypot(cur[0] - start[0], cur[1] - start[1])
                if remain <= self._tol_lin:
                    break
                self._send_vel(
                    sign * pm.approach_speed(
                        remain, pm.K_LIN, pm.V_MIN, pm.V_MAX),
                    0.0)
                time.sleep(0.02)
        finally:
            self._halt()

    def _nudge_ang(self, dtheta):
        """dtheta[rad] 만큼 제자리 회전. 목표를 지나치면 되돌리지 않고 멈춘다.

        되돌리면 그 되돌림이 또 지나치고… 를 반복하는 것이 헌팅이다. 1° 안쪽 오차는
        사진에 보이지 않으므로, 지나쳤으면 그냥 멈추는 편이 낫다.
        """
        start = self._pose(self._odom_frame)
        if start is None:
            return
        target = pm.normalize_angle(start[2] + dtheta)
        sign0 = 1.0 if dtheta > 0 else -1.0
        deadline = time.monotonic() + self._refine_timeout
        try:
            while time.monotonic() < deadline:
                cur = self._pose(self._odom_frame)
                if cur is None:
                    break
                err = pm.normalize_angle(target - cur[2])
                if abs(err) <= self._tol_ang or pm.overshot(err, sign0):
                    break
                self._send_vel(
                    0.0,
                    sign0 * pm.approach_speed(
                        err, pm.K_ANG, pm.W_MIN, pm.W_MAX))
                time.sleep(0.02)
        finally:
            self._halt()

    # ------------------------------------------------------------------ #
    # 정밀 주행 ③ 출발 정렬 / 정밀 조준
    # ------------------------------------------------------------------ #
    def _align_to(self, target_yaw, waypoint_id):
        """map 기준 target_yaw 로 제자리 회전. Nav2 에 목표를 주기 직전에 부른다.

        RPP 는 경로 시작 방향과 현재 방향이 rotate_to_heading_min_angle(20°) 이상
        벌어지면 **출발 전에도** 제자리 회전을 하는데, 그것도 등속이라 헌팅한다.
        먼저 우리가 감속하며 돌려 두면 RPP 는 회전할 일이 없어진다.

        두 번까지 반복하는 이유: 한 번의 회전이 tolerance 직전에서 멈추는 일이 있어
        (지나침 판정으로 조기 종료) 남은 각도를 한 번 더 줄인다.
        """
        for attempt in range(2):
            p = self._pose(self._global_frame)
            if p is None:
                return
            err = pm.normalize_angle(target_yaw - p[2])
            if abs(err) <= self._tol_ang:
                return
            if attempt == 0:
                self.get_logger().info(
                    f'waypoint={waypoint_id} 출발정렬 {math.degrees(err):+.1f}°')
            self._nudge_ang(err)

    def _refine(self, wp):
        """촬영 지점에서 목표 좌표·yaw 로 좁힌다. 보정 전/후 오차를 로그로 남긴다.

        회전 → 전후진 → 회전 순서다. 전후진하면 방향이 조금 틀어지므로 회전으로 끝낸다.
        횡(lat) 오차는 차동구동(옆으로 못 감)이라 고칠 수 없다 — 재서 로그로만 남긴다.

        크게 어긋난 경우(refine_max_fix_lin_m 초과)에는 손대지 않는다. 그 정도면
        Nav2 가 엉뚱한 자리에 세운 것이라 앞뒤로 미는 것이 위험할 수 있고, 원인은
        보정이 아니라 경로·좌표 쪽에 있기 때문이다. 경고만 남기고 그대로 찍는다.
        """
        wid = int(wp.waypoint_id)
        if not self._precision:
            return
        target = (float(wp.x), float(wp.y), float(wp.yaw))
        before = self._error(target)
        if before is None:
            self.get_logger().warn(
                f'waypoint={wid} {self._global_frame} TF 없음 → 정밀 조준 생략'
                f'(촬영은 그대로 진행)')
            return
        if abs(before[0]) > self._max_fix_lin:
            self.get_logger().warn(
                f'waypoint={wid} 도착 오차가 한계 초과 {_fmt_err(before)} → '
                f'전후진 보정 생략(회전만). 좌표·경로 점검 필요')

        for _ in range(self._refine_rounds):
            e = self._error(target)
            if e is None:
                break
            if abs(e[2]) > self._tol_ang:
                self._nudge_ang(e[2])
            e = self._error(target)
            if e is None:
                break
            if self._tol_lin < abs(e[0]) <= self._max_fix_lin:
                self._nudge_lin(e[0])

        e = self._error(target)      # 전후진 뒤 틀어진 방향 마무리
        if e is not None and abs(e[2]) > self._tol_ang:
            self._nudge_ang(e[2])

        self.get_logger().info(f'waypoint={wid} 도착 {_fmt_err(before)}')
        self.get_logger().info(f'waypoint={wid} 보정 {_fmt_err(self._error(target))}')


def main(args=None):
    rclpy.init(args=args)
    node = NavigateServer()
    # 중첩 액션 콜백이 서로를 막지 않도록 다중 스레드 executor 로 spin.
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
