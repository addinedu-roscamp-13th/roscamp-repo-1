#!/usr/bin/env python3
"""RP-113  E1/E2: DdaGo(주행 로봇) Navigate Action 서버 — 경로 배열 순차 주행.

ACS가 **예약을 확보한 구간까지**를 Waypoint 배열로 한 번에 하달하면(E1 6, E2 20),
로봇은 그 배열을 앞에서부터 순서대로 Nav2 로 주행한다. 로봇은 예약도 순찰 지점도
모르고, 받은 배열을 소화할 뿐이다.

이 노드가 맡는 역할:
  * Navigate 액션의 **서버**              (DCS → DdaGo, /ddago/navigate)
  * Nav2 NavigateToPose/Spin 액션의 **클라이언트** (DdaGo → Nav2)
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

**직전 원소와 좌표가 같은 waypoint 는 주행이 아니라 제자리 회전으로 처리한다**(E2 20-1).
DdaGo 의 순찰 카메라는 한쪽에 고정되어 통로 한쪽만 찍으므로, 양쪽을 다 찍어야 하는
지점은 ACS 가 x·y 는 같고 yaw 만 반대인 항목을 연달아 2개 넣어 하달한다. 로봇은 짝
개념을 모르고, 그저 "좌표가 같으면 돌기만 한다"는 규칙만 안다.

**촬영은 capture == true 인 노드에서만 한다**(E2 3단계). 나머지 노드는 통과만 한다.
어디서 찍을지는 ACS 가 정해서 플래그로 알려주며(`capture = 순찰지점 AND 미방문`),
로봇은 그 플래그만 본다. 분석 요청은 응답을 기다리지 않고 던져(fire-and-forget)
다음 waypoint 주행을 막지 않는다.

파라미터:
  robot_id               (str)   로그 표기용 로봇 식별자          기본 'dg_01'
  nav2_action            (str)   Nav2 주행 액션 이름(상대)        기본 'navigate_to_pose'
  spin_action            (str)   Nav2 제자리회전 액션 이름(상대)   기본 'spin'
  capture_service        (str)   카메라 노드 촬영 서비스(절대)     기본 '/ddago/capture_frame'
  analyze_service        (str)   DCS 분석 서비스(절대)            기본 '/dg/analyze_frame'
                                 (로봇 공용이라 네임스페이스 안 붙임 → 절대이름)
  arrival_settle_sec     (float) 도착 후 잔상 방지 정지 대기       기본 0.3
  capture_timeout_sec    (float) CaptureFrame 응답 대기 상한       기본 5.0
  nav2_wait_sec          (float) Nav2 서버/goal 수락 대기          기본 10.0
  nav2_result_timeout_sec(float) waypoint 1개당 결과 대기 상한     기본 300.0
  spin_time_allowance_sec(float) 제자리 회전 1회 제한 시간         기본 15.0
"""
import math
import threading
import time

from action_msgs.msg import GoalStatus
from automato_interfaces.action import Navigate
from automato_interfaces.srv import AnalyzeFrame, CaptureFrame
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

# Result.last_waypoint_id: 배열의 첫 노드에도 도달하지 못했음을 뜻하는 값.
# ACS 는 세그먼트 경로에 없는 값을 받으면 "세그먼트에 진입조차 못 했다"로 보고
# 진입 지점 기준으로 재계획한다(_segment_progress). wps[0] 을 보내면 "첫 노드까진
# 갔다"고 오해해 엉뚱한 통로를 막힘으로 판정하므로 반드시 이 값을 쓴다.
NO_WAYPOINT_REACHED = -1

# 직전 waypoint 와 "같은 자리"로 볼 허용 오차(m). ACS 는 짝 노드에 DB 의 같은 x·y 를
#그대로 실어 보내므로 원래는 정확히 일치하지만, float 왕복 오차를 감안해 1cm 를 둔다.
SAME_POSITION_TOL_M = 0.01


def _yaw_from_quaternion(q):
    """쿼터니언(x,y,z,w) → yaw(rad). tf 의존 없이 Z축 회전만 뽑는다."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _quaternion_from_yaw(yaw):
    """yaw(rad) → 쿼터니언 (z, w). Z축 회전만 있으므로 x=y=0 이다."""
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def _normalize_angle(a):
    """각도를 -pi ~ pi 로 접는다.

    Nav2 Spin 의 target_yaw 는 **절대 방향이 아니라 지금 방향에서 얼마나 돌지**(상대
    회전각)다. 짝 노드는 yaw 가 정반대(예: 1.57 → -1.57)라 그대로 빼면 -3.14 가 되고,
    접지 않으면 로봇이 먼 쪽으로 크게 돌아가는 경로를 고를 수 있다.
    """
    return math.atan2(math.sin(a), math.cos(a))


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
        self.declare_parameter('spin_action', 'spin')
        self.declare_parameter('capture_service', '/ddago/capture_frame')
        self.declare_parameter('analyze_service', '/dg/analyze_frame')
        self.declare_parameter('arrival_settle_sec', 0.3)
        self.declare_parameter('nav2_wait_sec', 10.0)
        self.declare_parameter('nav2_result_timeout_sec', 300.0)
        self.declare_parameter('spin_time_allowance_sec', 15.0)
        self.declare_parameter('capture_timeout_sec', 5.0)

        self._robot_id = self.get_parameter('robot_id').value
        self._nav2_action = self.get_parameter('nav2_action').value
        self._spin_action = self.get_parameter('spin_action').value
        capture_service = self.get_parameter('capture_service').value
        analyze_service = self.get_parameter('analyze_service').value
        self._settle_sec = float(self.get_parameter('arrival_settle_sec').value)
        self._nav2_wait = float(self.get_parameter('nav2_wait_sec').value)
        self._nav2_result_timeout = float(
            self.get_parameter('nav2_result_timeout_sec').value)
        self._spin_time_allowance = float(
            self.get_parameter('spin_time_allowance_sec').value)
        self._capture_timeout = float(
            self.get_parameter('capture_timeout_sec').value)

        # 서버 실행(블로킹) 중에도 Nav2 콜백이 처리되도록 모든 통신을
        # ReentrantCallbackGroup 에 넣는다(main 의 MultiThreadedExecutor 와 짝).
        self._cb = ReentrantCallbackGroup()

        # --- 상태 ---
        self._nav_client = None     # Nav2 NavigateToPose 클라이언트 (지연 생성)
        self._spin_client = None    # Nav2 Spin 클라이언트 (지연 생성)
        self._last_pose = None      # Nav2 가 보고한 최신 (x, y, yaw). 도착 피드백에 실음
        self._pending_analyze = []  # call_async future 보관(GC 방지)

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
            f'Nav2={self._nav2_action}/{self._spin_action}, '
            f'촬영={capture_service}, 분석={analyze_service}')

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

            # 직전 원소와 좌표가 같으면 이동 없이 방향만 바꾼다(E2 20-1 짝 노드).
            # 배열 안에서만 비교하면 된다 — ACS 는 짝을 항상 **연달아 2개** 넣어
            # 보내므로, 배열의 첫 원소가 제자리 회전 대상이 되는 경우는 없다.
            prev = wps[idx - 1] if idx > 0 else None
            if prev is not None and self._same_position(prev, wp):
                self.get_logger().info(
                    f'제자리 회전 task={task_id} [{idx + 1}/{len(wps)}] '
                    f'waypoint={int(wp.waypoint_id)} yaw={wp.yaw:.2f} '
                    f'(직전 {int(prev.waypoint_id)} 과 같은 자리 → 방향만 전환)')
                code = self._spin_to(goal_handle, wp)
            else:
                self.get_logger().info(
                    f'주행 시작 task={task_id} [{idx + 1}/{len(wps)}] '
                    f'waypoint={int(wp.waypoint_id)} '
                    f'({wp.x:.2f},{wp.y:.2f}) yaw={wp.yaw:.2f}')
                code = self._drive_to(goal_handle, wp)
            if code != 0:
                self.get_logger().warn(
                    f'주행/회전 실패·중단 task={task_id} '
                    f'waypoint={int(wp.waypoint_id)} code={code} → 구간 종료 '
                    f'(마지막 도달 노드={last_wp})')
                break

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
    def _same_position(a, b, tol=SAME_POSITION_TOL_M):
        """두 waypoint 가 같은 자리인가(= 이동 없이 방향만 바꾸면 되는가)."""
        return abs(a.x - b.x) <= tol and abs(a.y - b.y) <= tol

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
        # 좌표는 Nav2 가 보고한 실측 위치를 쓴다. 아직 한 번도 못 받았으면
        # 목표 좌표로 대신한다(도착했으므로 목표와 거의 같다).
        if self._last_pose is not None:
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

        nav_goal = NavigateToPose.Goal()
        ps = PoseStamped()
        ps.header.frame_id = 'map'
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = float(wp.x)
        ps.pose.position.y = float(wp.y)
        # 도착 방향(yaw)까지 지정한다. 카메라가 한쪽에 고정되어 있어 어느 쪽을 보고
        # 서는지가 곧 무엇을 찍는지이므로, 방향을 Nav2 에 맡기면 베드 반대쪽을 찍는다.
        ps.pose.orientation.z, ps.pose.orientation.w = _quaternion_from_yaw(
            float(wp.yaw))
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
    # 제자리 회전: Nav2 Spin 으로 방향만 전환 (E2 20-1 짝 노드)
    # ------------------------------------------------------------------ #
    def _spin_to(self, goal_handle, wp):
        # 지연 임포트: nav2_msgs 없는 개발환경에서도 이 모듈이 임포트되게.
        from builtin_interfaces.msg import Duration
        from nav2_msgs.action import Spin
        from rclpy.action import ActionClient

        if self._spin_client is None:
            self._spin_client = ActionClient(
                self, Spin, self._spin_action, callback_group=self._cb)

        if not self._spin_client.wait_for_server(timeout_sec=self._nav2_wait):
            self.get_logger().warn(
                f'Nav2 {self._spin_action} 서버 없음 → 실패(1)')
            return 1

        # Spin.target_yaw 는 **상대 회전각**이라 (목표 - 현재)를 접어서 넣는다.
        # 현재 방향은 직전 주행에서 Nav2 가 보고한 실측 yaw 를 쓴다.
        current_yaw = self._last_pose[2] if self._last_pose is not None else 0.0
        delta = _normalize_angle(float(wp.yaw) - current_yaw)

        sec = int(self._spin_time_allowance)
        spin_goal = Spin.Goal()
        spin_goal.target_yaw = float(delta)
        spin_goal.time_allowance = Duration(
            sec=sec, nanosec=int((self._spin_time_allowance - sec) * 1e9))
        self.get_logger().info(
            f'Spin 요청: 현재 yaw={current_yaw:.2f} → 목표 {float(wp.yaw):.2f} '
            f'(상대 {delta:.2f} rad)')

        spin_handle = _spin_wait(
            self._spin_client.send_goal_async(spin_goal), self._nav2_wait)
        if spin_handle is None or not spin_handle.accepted:
            self.get_logger().warn('Spin goal 거부/타임아웃 → 실패(1)')
            return 1

        def _check_cancel():
            if goal_handle.is_cancel_requested:
                spin_handle.cancel_goal_async()

        result_resp = _spin_wait(
            spin_handle.get_result_async(), self._nav2_result_timeout,
            on_wait=_check_cancel)
        if result_resp is None:
            self.get_logger().warn('Spin 결과 타임아웃 → 실패(1)')
            return 1

        status = result_resp.status
        if status == GoalStatus.STATUS_SUCCEEDED:
            # Spin 은 NavigateToPose 와 달리 현재 pose 피드백을 주지 않는다(회전량만
            # 알려준다). 자리는 그대로이고 방향만 목표대로 바뀌었으므로 캐시를 손으로
            # 갱신한다. 빠뜨리면 다음 Spin 이 낡은 yaw 를 기준으로 상대각을 계산해
            # 회전 오차가 누적된다.
            x, y = ((self._last_pose[0], self._last_pose[1])
                    if self._last_pose is not None
                    else (float(wp.x), float(wp.y)))
            self._last_pose = (x, y, float(wp.yaw))
            return 0
        if status == GoalStatus.STATUS_CANCELED:
            return 2
        return 1   # ABORTED 등


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
