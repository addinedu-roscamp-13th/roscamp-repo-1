#!/usr/bin/env python3
"""RP-78 테스트 스탠드인 ② — Patrol 액션 브릿지 (HQ+DdaGo 주행 대역의 최소 흉내).

⚠️ 실제 DG Control Service 가 아니다. ACS가 보내는 /<robot_id>/patrol (Patrol 액션)을
   받아줄 서버가 아직 없어서, 그 최소 기능만 흉내내는 '테스트 전용' 노드다.
   (실제 HQ/DdaGo 액션 서버가 생기면 이 파일은 버린다.)

제공: /<robot_id>/patrol  (automato_interfaces/action/Patrol 서버)
  Goal:   task_id + 단일 WaypointGoal(waypoint_id, x, y)
  Result: result_code(0 성공/1 실패·막힘/2 중단) + message
  Feedback: current_waypoint_id + current_x/y/yaw

두 가지 모드(파라미터 mode):
  - sim  : Nav2 를 부르지 않고 sim_seconds 초 뒤 '도착(0)' 응답. 로봇은 안 움직인다.
           → ACS의 디스패치·통로예약·DB 상태전이·스냅샷 로직을 로봇 세워두고 안전 검증.
  - nav2 : 로봇의 Nav2 navigate_to_pose 액션을 실제로 호출해 그 좌표까지 주행시킨다.
           → 로봇이 진짜 움직인다. (로봇에 Nav2 가 떠 있어야 함)

이름 규칙: 네임스페이스로 로봇을 구분한다(핑키/텔레메트리와 동일).
  __ns:=/dg_01 로 실행하면 서버는 /dg_01/patrol, Nav2 클라이언트는 /dg_01/navigate_to_pose.

실행 (🤖 각 로봇 RPi5, ROS + automato_ws 소싱 후):
  # 실제 주행 (Nav2 필요)
  python3 patrol_bridge.py --ros-args -r __ns:=/dg_01 -p mode:=nav2 -p robot_id:=dg_01
  # 가짜 도착 (로봇 안 움직임, 로직만)
  python3 patrol_bridge.py --ros-args -r __ns:=/dg_01 -p mode:=sim  -p robot_id:=dg_01
"""
import threading
import time

from action_msgs.msg import GoalStatus
from automato_interfaces.action import Patrol
import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node


def _spin_wait(future, timeout, poll=0.5, on_wait=None):
    """executor(백그라운드 spin)가 완료할 future를 다른 스레드에서 기다린다.

    on_wait: poll 간격마다 호출되는 콜백(예: 취소 요청 확인). 타임아웃/예외 시 None.
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


class PatrolBridge(Node):
    def __init__(self, **kwargs):
        super().__init__("patrol_bridge", **kwargs)

        self.declare_parameter("mode", "sim")            # 'sim' | 'nav2'
        self.declare_parameter("robot_id", "dg_01")      # 로그 표기용
        self.declare_parameter("sim_seconds", 5.0)       # sim 모드 가짜 이동 시간
        # sim 모드에서 '막힘(result_code=1)'으로 응답할 waypoint_id 들(콤마구분, 예: "3,5").
        #   → 막힘→블랙리스트→우회/건너뜀 로직을 로봇 없이 검증할 때 쓴다.
        #   (빈 리스트 파라미터는 타입추론이 안 돼 문자열로 받는다)
        self.declare_parameter("fail_waypoint_ids", "")
        self.declare_parameter("nav2_action", "navigate_to_pose")
        self.declare_parameter("nav2_wait_sec", 10.0)    # Nav2 서버/수락 대기
        self.declare_parameter("nav2_result_timeout_sec", 300.0)

        self._mode = self.get_parameter("mode").value
        self._robot_id = self.get_parameter("robot_id").value
        self._sim_seconds = float(self.get_parameter("sim_seconds").value)
        raw_fail = self.get_parameter("fail_waypoint_ids").value or ""
        self._fail_wps = {int(x) for x in raw_fail.split(",") if x.strip()}
        self._nav2_action = self.get_parameter("nav2_action").value
        self._nav2_wait = float(self.get_parameter("nav2_wait_sec").value)
        self._nav2_result_timeout = float(
            self.get_parameter("nav2_result_timeout_sec").value)

        self._cb = ReentrantCallbackGroup()   # 서버 실행 중 Nav2 콜백도 처리되도록
        self._nav_client = None               # nav2 모드에서 지연 생성

        # Patrol 액션 서버 (상대명 'patrol' → 네임스페이스로 /<robot_id>/patrol)
        self._server = ActionServer(
            self, Patrol, "patrol",
            execute_callback=self._execute,
            cancel_callback=lambda _gh: CancelResponse.ACCEPT,
            callback_group=self._cb,
        )
        self.get_logger().info(
            f"[TEST] Patrol 브릿지 준비: mode={self._mode} robot_id={self._robot_id} "
            f"→ 서버 patrol. ※ 실제 DG Control Service 아님(테스트 스탠드인)")
        if self._mode == "nav2":
            self.get_logger().info(
                f"[TEST] nav2 모드 — Goal 을 {self._nav2_action} 로 넘겨 실제 주행")

    # ------------------------------------------------------------------ #
    # Patrol 실행 콜백: 단일 waypoint 이동을 처리하고 result_code 반환
    # ------------------------------------------------------------------ #
    def _execute(self, goal_handle):
        req = goal_handle.request
        wp = req.waypoint
        self.get_logger().info(
            f"[TEST] Patrol 수신 task={req.task_id} waypoint={wp.waypoint_id} "
            f"({wp.x:.2f},{wp.y:.2f}) mode={self._mode}")

        if self._mode == "nav2":
            code = self._run_nav2(goal_handle, wp)
        else:
            code = self._run_sim(goal_handle, wp)

        result = Patrol.Result()
        result.result_code = int(code)
        result.message = f"{self._mode} bridge → code {code}"
        if code == 0:
            goal_handle.succeed()
        elif code == 2:
            goal_handle.canceled()
        else:
            goal_handle.abort()
        self.get_logger().info(
            f"[TEST] Patrol 종료 task={req.task_id} waypoint={wp.waypoint_id} "
            f"result_code={code}")
        return result

    def _publish_feedback(self, goal_handle, wp, x=None, y=None, yaw=0.0):
        fb = Patrol.Feedback()
        fb.current_waypoint_id = int(wp.waypoint_id)
        fb.current_x = float(wp.x if x is None else x)
        fb.current_y = float(wp.y if y is None else y)
        fb.current_yaw = float(yaw)
        goal_handle.publish_feedback(fb)

    # ------------------------------- sim ------------------------------- #
    def _run_sim(self, goal_handle, wp):
        """Nav2 없이 sim_seconds 뒤 도착(0). 취소 요청 오면 중단(2).

        fail_waypoint_ids 에 든 waypoint 는 막힘(1)으로 응답 → 우회/건너뜀 로직 검증용.
        """
        if int(wp.waypoint_id) in self._fail_wps:
            self.get_logger().warn(
                f"[TEST] sim 막힘 시뮬 waypoint={wp.waypoint_id} → 실패(1)")
            return 1
        steps = max(1, int(round(self._sim_seconds)))
        for _ in range(steps):
            if goal_handle.is_cancel_requested:
                return 2
            self._publish_feedback(goal_handle, wp)
            time.sleep(1.0)
        self._publish_feedback(goal_handle, wp)
        return 0

    # ------------------------------- nav2 ------------------------------ #
    def _run_nav2(self, goal_handle, wp):
        """로봇 Nav2 navigate_to_pose 로 실제 주행. 결과를 Patrol result_code 로 매핑."""
        # 지연 임포트: sim 전용 환경(nav2_msgs 없음)에서도 이 파일이 임포트되게.
        from geometry_msgs.msg import PoseStamped
        from nav2_msgs.action import NavigateToPose
        from rclpy.action import ActionClient

        if self._nav_client is None:
            self._nav_client = ActionClient(
                self, NavigateToPose, self._nav2_action, callback_group=self._cb)

        if not self._nav_client.wait_for_server(timeout_sec=self._nav2_wait):
            self.get_logger().warn(
                f"[TEST] Nav2 {self._nav2_action} 서버 없음 → 실패(1)")
            return 1

        nav_goal = NavigateToPose.Goal()
        ps = PoseStamped()
        ps.header.frame_id = "map"
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = float(wp.x)
        ps.pose.position.y = float(wp.y)
        ps.pose.orientation.w = 1.0        # yaw 0 (목적지 방향은 Nav2가 알아서)
        nav_goal.pose = ps

        def _fb(nav_fb):
            p = nav_fb.feedback.current_pose.pose
            self._publish_feedback(goal_handle, wp, p.position.x, p.position.y)

        nav_handle = _spin_wait(
            self._nav_client.send_goal_async(nav_goal, feedback_callback=_fb),
            self._nav2_wait)
        if nav_handle is None or not nav_handle.accepted:
            self.get_logger().warn("[TEST] Nav2 goal 거부/타임아웃 → 실패(1)")
            return 1

        # 결과 대기(취소 요청 오면 Nav2 goal도 취소)
        def _check_cancel():
            if goal_handle.is_cancel_requested:
                nav_handle.cancel_goal_async()

        result_resp = _spin_wait(
            nav_handle.get_result_async(), self._nav2_result_timeout,
            on_wait=_check_cancel)
        if result_resp is None:
            self.get_logger().warn("[TEST] Nav2 결과 타임아웃 → 실패(1)")
            return 1

        status = result_resp.status
        if status == GoalStatus.STATUS_SUCCEEDED:
            return 0
        if status == GoalStatus.STATUS_CANCELED:
            return 2
        return 1   # ABORTED 등


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PatrolBridge()
    # 액션 서버 실행 중 Nav2 클라이언트 콜백도 병행 처리하려면 멀티스레드 executor.
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
