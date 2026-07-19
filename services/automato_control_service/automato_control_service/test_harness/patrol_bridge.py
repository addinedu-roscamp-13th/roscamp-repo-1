#!/usr/bin/env python3
"""RP-78 테스트 스탠드인 ② — Navigate 액션 브릿지 (DG Control Service+DdaGo 주행 대역의 최소 흉내).

⚠️ 실제 DG Control Service 가 아니다. ACS가 보내는 /<robot_id>/navigate (Navigate 액션)을
   받아줄 서버가 아직 없어서, 그 최소 기능만 흉내내는 '테스트 전용' 노드다.
   (실제 DCS/DdaGo 액션 서버가 생기면 이 파일은 버린다.)
   파일명의 'patrol'은 지원 시나리오(주간 순찰)를 뜻하며, 액션은 Navigate 다.

제공: /<robot_id>/navigate  (automato_interfaces/action/Navigate 서버)
  Goal:     task_id + Waypoint[] waypoints(예약 확보된 구간까지의 경로 배열)
  Result:   result_code(0 성공/1 실패·막힘/2 중단) + last_waypoint_id + message
  Feedback: current_waypoint_id + waypoint_index + current_x/y/yaw

  last_waypoint_id 가 핵심: 실패/중단 시 '실제 도달한 마지막 노드'를 돌려줘야
  ACS 의 _segment_progress 가 그 지점부터 우회를 재계획한다. 하나도 못 갔으면 -1.

두 가지 모드(파라미터 mode):
  - sim  : Nav2 없이 배열의 각 waypoint 를 순서대로 '도달'시킨다(로봇 안 움직임).
           → ACS의 세그먼트 디스패치·통로예약·룩어헤드·DB 상태전이를 로봇 세워두고 검증.
  - nav2 : 각 waypoint 를 로봇 Nav2 navigate_to_pose 로 실제 주행(로봇에 Nav2 필요).

이름 규칙: 네임스페이스로 로봇을 구분한다(핑키/텔레메트리와 동일).
  __ns:=/dg_01 로 실행하면 서버는 /dg_01/navigate, Nav2 클라이언트는 /dg_01/navigate_to_pose.

실행 (🤖 각 로봇 RPi5, ROS + automato_ws 소싱 후):
  # 실제 주행 (Nav2 필요)
  python3 patrol_bridge.py --ros-args -r __ns:=/dg_01 -p mode:=nav2 -p robot_id:=dg_01
  # 가짜 도달 (로봇 안 움직임, 로직만; 빠르게 보려면 -p sim_seconds:=1)
  python3 patrol_bridge.py --ros-args -r __ns:=/dg_01 -p mode:=sim  -p robot_id:=dg_01
"""
import math
import threading
import time

from action_msgs.msg import GoalStatus
from automato_interfaces.action import Navigate
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
        self.declare_parameter("sim_seconds", 5.0)       # sim 모드 waypoint 당 가짜 이동 시간
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

        # Navigate 액션 서버 (상대명 'navigate' → 네임스페이스로 /<robot_id>/navigate)
        self._server = ActionServer(
            self, Navigate, "navigate",
            execute_callback=self._execute,
            cancel_callback=lambda _gh: CancelResponse.ACCEPT,
            callback_group=self._cb,
        )
        self.get_logger().info(
            f"[TEST] Navigate 브릿지 준비: mode={self._mode} robot_id={self._robot_id} "
            f"→ 서버 navigate. ※ 실제 DG Control Service 아님(테스트 스탠드인)")
        if self._mode == "nav2":
            self.get_logger().info(
                f"[TEST] nav2 모드 — 각 waypoint 를 {self._nav2_action} 로 넘겨 실제 주행")

    # ------------------------------------------------------------------ #
    # Navigate 실행 콜백: waypoints 배열을 순서대로 처리하고 (code, last_wp) 반환
    # ------------------------------------------------------------------ #
    def _execute(self, goal_handle):
        req = goal_handle.request
        waypoints = list(req.waypoints)
        ids = [int(w.waypoint_id) for w in waypoints]
        self.get_logger().info(
            f"[TEST] Navigate 수신 task={req.task_id} waypoints={ids} mode={self._mode}")

        if not waypoints:
            code, last_id = 0, -1              # 빈 배열은 '할 일 없음' 성공
        elif self._mode == "nav2":
            code, last_id = self._run_nav2(goal_handle, waypoints)
        else:
            code, last_id = self._run_sim(goal_handle, waypoints)

        result = Navigate.Result()
        result.result_code = int(code)
        result.last_waypoint_id = int(last_id)
        result.message = f"{self._mode} bridge → code {code}, last_wp {last_id}"
        if code == 0:
            goal_handle.succeed()
        elif code == 2:
            goal_handle.canceled()
        else:
            goal_handle.abort()
        self.get_logger().info(
            f"[TEST] Navigate 종료 task={req.task_id} code={code} last_wp={last_id}")
        return result

    def _publish_feedback(self, goal_handle, wp, idx, x=None, y=None, yaw=None):
        fb = Navigate.Feedback()
        fb.current_waypoint_id = int(wp.waypoint_id)
        fb.waypoint_index = int(idx)
        fb.current_x = float(wp.x if x is None else x)
        fb.current_y = float(wp.y if y is None else y)
        fb.current_yaw = float(wp.yaw if yaw is None else yaw)
        goal_handle.publish_feedback(fb)

    # ------------------------------- sim ------------------------------- #
    def _run_sim(self, goal_handle, waypoints):
        """Nav2 없이 배열의 각 waypoint 를 순서대로 '도달'시킨다.

        fail_waypoint_ids 에 든 waypoint 를 만나면 그 직전까지 도달로 보고 막힘(1)을 반환
        → ACS 의 last_waypoint_id 기반 우회(_segment_progress)를 로봇 없이 검증.
        반환: (result_code, last_waypoint_id). 아직 하나도 못 갔으면 last=-1.
        """
        last_reached = -1
        steps = max(1, int(round(self._sim_seconds)))   # waypoint 당 이동 시간(초)
        for idx, wp in enumerate(waypoints):
            if int(wp.waypoint_id) in self._fail_wps:
                self.get_logger().warn(
                    f"[TEST] sim 막힘 시뮬 waypoint={wp.waypoint_id} "
                    f"→ 실패(1), 마지막 도달={last_reached}")
                return 1, last_reached
            for _ in range(steps):                       # 1초 간격으로 취소 확인
                if goal_handle.is_cancel_requested:
                    return 2, last_reached
                time.sleep(1.0)
            last_reached = int(wp.waypoint_id)           # 이 waypoint 도달 완료
            self._publish_feedback(goal_handle, wp, idx)
        return 0, last_reached                           # 배열 끝까지 도달

    # ------------------------------- nav2 ------------------------------ #
    def _run_nav2(self, goal_handle, waypoints):
        """배열의 각 waypoint 를 Nav2 navigate_to_pose 로 순서대로 실제 주행.

        도중 한 waypoint 라도 실패/중단이면 거기서 멈추고 직전 도달 id 를 함께 돌려준다.
        반환: (result_code, last_waypoint_id).
        """
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
            return 1, -1

        last_reached = -1
        for idx, wp in enumerate(waypoints):
            code = self._nav2_one(goal_handle, wp, idx, PoseStamped, NavigateToPose)
            if code != 0:
                return code, last_reached                # 실패/중단 → 직전 도달까지
            last_reached = int(wp.waypoint_id)
        return 0, last_reached

    def _nav2_one(self, goal_handle, wp, idx, PoseStamped, NavigateToPose):
        """단일 waypoint 를 Nav2 로 주행. 반환 code: 0 성공 / 1 실패 / 2 중단."""
        nav_goal = NavigateToPose.Goal()
        ps = PoseStamped()
        ps.header.frame_id = "map"
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = float(wp.x)
        ps.pose.position.y = float(wp.y)
        ps.pose.orientation.z = math.sin(float(wp.yaw) / 2.0)   # yaw → 쿼터니언(z,w)
        ps.pose.orientation.w = math.cos(float(wp.yaw) / 2.0)
        nav_goal.pose = ps

        def _fb(nav_fb):
            p = nav_fb.feedback.current_pose.pose
            self._publish_feedback(goal_handle, wp, idx, p.position.x, p.position.y)

        nav_handle = _spin_wait(
            self._nav_client.send_goal_async(nav_goal, feedback_callback=_fb),
            self._nav2_wait)
        if nav_handle is None or not nav_handle.accepted:
            self.get_logger().warn(
                f"[TEST] Nav2 goal 거부/타임아웃 waypoint={wp.waypoint_id} → 실패(1)")
            return 1

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
