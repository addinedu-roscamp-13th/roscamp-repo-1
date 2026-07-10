#!/usr/bin/env python3
"""RP-76  E2: DdaGo(주행 로봇) Patrol Action 서버 + 웨이포인트 촬영·분석요청.

ACS/HQ가 `Patrol` 액션으로 하달하는 **단일 waypoint**를 받아 인접 노드까지 Nav2로
주행하고, 도착 시 정지 상태에서 RGB 프레임 1장을 캡처한 뒤 HQ에 분석을 요청한다.
분석·저장은 비동기라 로봇 이동을 막지 않는다(fire-and-forget).

이 노드는 두 역할을 동시에 한다:
  * Patrol 액션의 **서버**            (ACS/HQ → DdaGo,  /dg_01/ddago/patrol)
  * Nav2 NavigateToPose 액션의 **클라이언트** (DdaGo → Nav2, /dg_01/navigate_to_pose)
  * AnalyzeFrame 서비스의 **클라이언트**      (DdaGo → HQ,   /dg/analyze_frame)
  * 카메라 이미지 토픽의 **구독자**            (드라이버 → DdaGo, image_raw)

전제: ACS는 순찰 지점만 goal로 하달한다(중간 이동은 Nav2가 알아서 지나감).
따라서 Patrol goal 하나 = 순찰 지점 하나 → 도착할 때마다 무조건 촬영한다.

동시성: execute 콜백이 Nav2 결과를 기다리며 블로킹하는 동안에도 카메라 구독/Nav2
피드백/분석응답 콜백이 계속 돌아야 하므로 MultiThreadedExecutor +
ReentrantCallbackGroup 을 쓴다(main() 참고). Nav2 결과 대기는 patrol_bridge 에서
검증된 _spin_wait 폴링 패턴을 그대로 채택한다.

도착 시 순서 (티켓 4절):
  도착·정지 → settle 대기(~300ms) → 프레임 grab
            → (병렬) result_code=0 반환  &  analyze_frame 호출
(프레임 grab 은 멤버변수 스냅샷이라 즉시 끝나므로 "프레임 확보 후 반환"이 보장되고,
 반환이 늦어지지 않는다. 분석요청은 응답을 기다리지 않고 던진다.)

파라미터:
  robot_id               (str)   로그 표기용 로봇 식별자          기본 'dg_01'
  camera_topic           (str)   구독할 카메라 이미지 토픽(상대)   기본 'image_raw'
                                 (실기 토픽은 RP-85 bringup 후 launch 로 주입)
  nav2_action            (str)   Nav2 액션 이름(상대)             기본 'navigate_to_pose'
  analyze_service        (str)   HQ 분석 서비스(절대)             기본 '/dg/analyze_frame'
                                 (로봇 공용이라 네임스페이스 안 붙임 → 절대이름)
  arrival_settle_sec     (float) 도착 후 잔상 방지 정지 대기       기본 0.3
  nav2_wait_sec          (float) Nav2 서버/goal 수락 대기          기본 10.0
  nav2_result_timeout_sec(float) Nav2 결과 대기 상한              기본 300.0
"""
import math
import threading
import time

from action_msgs.msg import GoalStatus
from automato_interfaces.action import Patrol
from automato_interfaces.srv import AnalyzeFrame
import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


def _yaw_from_quaternion(q):
    """쿼터니언(x,y,z,w) → yaw(rad). tf 의존 없이 Z축 회전만 뽑는다."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


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


class PatrolServer(Node):
    def __init__(self, **kwargs):
        # **kwargs 는 테스트에서 parameter_overrides 등을 주입하기 위한 통로.
        super().__init__('patrol_server', **kwargs)

        # --- 파라미터 ---
        self.declare_parameter('robot_id', 'dg_01')
        self.declare_parameter('camera_topic', 'image_raw')
        self.declare_parameter('nav2_action', 'navigate_to_pose')
        self.declare_parameter('analyze_service', '/dg/analyze_frame')
        self.declare_parameter('arrival_settle_sec', 0.3)
        self.declare_parameter('nav2_wait_sec', 10.0)
        self.declare_parameter('nav2_result_timeout_sec', 300.0)

        self._robot_id = self.get_parameter('robot_id').value
        camera_topic = self.get_parameter('camera_topic').value
        self._nav2_action = self.get_parameter('nav2_action').value
        analyze_service = self.get_parameter('analyze_service').value
        self._settle_sec = float(self.get_parameter('arrival_settle_sec').value)
        self._nav2_wait = float(self.get_parameter('nav2_wait_sec').value)
        self._nav2_result_timeout = float(
            self.get_parameter('nav2_result_timeout_sec').value)

        # 서버 실행(블로킹) 중에도 Nav2 콜백·카메라 구독·분석응답이 처리되도록
        # 모든 통신을 ReentrantCallbackGroup 에 넣는다(main 의 MultiThreadedExecutor 와 짝).
        self._cb = ReentrantCallbackGroup()

        # --- 상태 ---
        self._latest_frame = None       # 카메라가 상시 갱신하는 최신 Image (도착 시 grab)
        self._nav_client = None         # Nav2 클라이언트 (지연 생성)
        self._pending_analyze = []      # call_async future 보관(GC 방지)

        # --- 카메라 상시 구독 ---
        # 타입은 sensor_msgs/Image 로 확정(어떤 USB 카메라 드라이버든 동일).
        # sensor QoS(best_effort)로 구독하면 드라이버가 reliable/best_effort 어느 쪽이든
        # 매칭된다. 콜백은 최신 프레임을 멤버변수에 저장만 한다.
        self.create_subscription(
            Image, camera_topic, self._image_cb, qos_profile_sensor_data,
            callback_group=self._cb)

        # --- 분석요청 서비스 클라이언트 ---
        self._analyze_client = self.create_client(
            AnalyzeFrame, analyze_service, callback_group=self._cb)

        # --- Patrol 액션 서버 (상대명 'ddago/patrol' → 네임스페이스로 /dg_01/ddago/patrol) ---
        self._server = ActionServer(
            self, Patrol, 'ddago/patrol',
            execute_callback=self._execute,
            cancel_callback=lambda _gh: CancelResponse.ACCEPT,
            callback_group=self._cb,
        )

        self.get_logger().info(
            f'Patrol 서버 준비됨: robot_id={self._robot_id} → 서버 ddago/patrol, '
            f'Nav2={self._nav2_action}, 카메라={camera_topic}, '
            f'분석={analyze_service}')

    # ------------------------------------------------------------------ #
    # 카메라 구독 콜백: 최신 프레임만 캐시
    # ------------------------------------------------------------------ #
    def _image_cb(self, msg):
        self._latest_frame = msg

    # ------------------------------------------------------------------ #
    # Patrol 실행 콜백: 단일 waypoint 주행 → 도착 촬영 → result_code 반환
    # ------------------------------------------------------------------ #
    def _execute(self, goal_handle):
        req = goal_handle.request
        wp = req.waypoint
        self.get_logger().info(
            f'Patrol 수신 task={req.task_id} waypoint={wp.waypoint_id} '
            f'({wp.x:.2f},{wp.y:.2f}) → 주행 시작')

        code = self._drive_to(goal_handle, wp)

        # 도착(0)일 때만 촬영·분석요청. 실패/취소는 촬영하지 않는다.
        if code == 0:
            self._capture_and_request(req.task_id, wp.waypoint_id)

        result = Patrol.Result()
        result.result_code = int(code)
        result.message = {
            0: 'arrived',
            1: 'failed/blocked',
            2: 'canceled',
        }.get(code, 'unknown')

        if code == 0:
            goal_handle.succeed()
        elif code == 2:
            goal_handle.canceled()
        else:
            goal_handle.abort()

        self.get_logger().info(
            f'Patrol 종료 task={req.task_id} waypoint={wp.waypoint_id} '
            f'result_code={code}')
        return result

    def _publish_feedback(self, goal_handle, wp, x=None, y=None, yaw=0.0):
        fb = Patrol.Feedback()
        fb.current_waypoint_id = int(wp.waypoint_id)
        fb.current_x = float(wp.x if x is None else x)
        fb.current_y = float(wp.y if y is None else y)
        fb.current_yaw = float(yaw)
        goal_handle.publish_feedback(fb)

    # ------------------------------------------------------------------ #
    # 주행: Nav2 NavigateToPose 로 인접 노드 1개까지. result_code(0/1/2) 반환
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
        # yaw 는 넣지 않음(카메라 좌측 고정, 회전 없음). 목적지 방향은 Nav2가 알아서.
        ps.pose.orientation.w = 1.0
        nav_goal.pose = ps

        # Nav2 피드백(current_pose) → Patrol 피드백으로 변환·중계
        def _fb(nav_fb):
            p = nav_fb.feedback.current_pose.pose
            self._publish_feedback(
                goal_handle, wp, p.position.x, p.position.y,
                _yaw_from_quaternion(p.orientation))

        nav_handle = _spin_wait(
            self._nav_client.send_goal_async(nav_goal, feedback_callback=_fb),
            self._nav2_wait)
        if nav_handle is None or not nav_handle.accepted:
            self.get_logger().warn('Nav2 goal 거부/타임아웃 → 실패(1)')
            return 1

        # 결과 대기. 상위(ACS)가 취소 요청하면 Nav2 goal 도 취소.
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
    # 도착 처리: settle 대기 → 프레임 grab → analyze_frame 호출(fire-and-forget)
    # ------------------------------------------------------------------ #
    def _capture_and_request(self, task_id, waypoint_id):
        # 정지 잔상 방지용 짧은 대기 후 최신 프레임 스냅샷.
        time.sleep(self._settle_sec)
        frame = self._latest_frame
        if frame is None:
            self.get_logger().warn(
                f'waypoint={waypoint_id} 도착했으나 카메라 프레임 미수신 → '
                f'분석요청 스킵(주행은 성공 처리). 카메라 토픽/드라이버 확인 필요')
            return
        self._request_analyze(task_id, waypoint_id, frame)

    def _request_analyze(self, task_id, waypoint_id, frame):
        """캡처한 원본 프레임을 /dg/analyze_frame 에 던진다(응답 안 기다림)."""
        if not self._analyze_client.service_is_ready():
            self.get_logger().warn(
                'analyze_frame 서비스 미준비 → 이번 프레임 분석요청 스킵 '
                '(HQ 분석 서버 확인 필요)')
            return

        request = AnalyzeFrame.Request()
        request.task_id = int(task_id)
        request.waypoint_id = int(waypoint_id)
        request.image = frame   # 원본 그대로 전달 (jpeg/base64 변환은 HQ 몫)

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


def main(args=None):
    rclpy.init(args=args)
    node = PatrolServer()
    # 중첩 액션/서비스 콜백이 서로를 막지 않도록 다중 스레드 executor 로 spin.
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
