#!/usr/bin/env python3
"""반사테이프 후진 도킹 — 단일 FSM 디버그 노드 (명령 하나로 도킹 전체 수행).

reflective_fsm.ReflectiveDockFsm(순수 FSM)을 얇게 감싼 standalone 노드다.
액션 없이 `ros2 run` 한 번으로 SNAP→…→CREEP 전 과정을 돌리고, 성공/실패를
exit code(0/1)로 돌려준다. 튜닝·현장 디버그용이다.

실운용(ACS 가 도킹을 지시하는 경로)은 reflective_dock_server.py(액션 서버)가
맡는다. 두 노드가 **같은 reflective_fsm 을 구동**하므로 로직 출처는 하나다
(드리프트 없음).

전제: 라이다(Standard)가 /scan → reflective_detector 가 /docking_marker_pose,
      베이스가 /odom 을 발행 중이어야 한다.
실행: ros2 run ddago_control reflective_dock --ros-args --params-file <robot>.yaml
      (다른 로봇: -p rear_offset_m:=0.068)
"""
import math
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry

from .reflective_fsm import ReflectiveDockFsm, RC_OK, WATCHDOG_SEC

RATE_HZ = 20


class ReflectiveDockNode(Node):
    def __init__(self):
        super().__init__("reflective_dock")
        self.declare_parameter("rear_offset_m", 0.10)
        self.declare_parameter("stop_gap_m", 0.02)
        rear = self.get_parameter("rear_offset_m").value
        gap = self.get_parameter("stop_gap_m").value
        self.fsm = ReflectiveDockFsm(rear_offset_m=rear, stop_gap_m=gap)

        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(PoseStamped, "/docking_marker_pose", self.on_pose, 10)
        self.create_subscription(Odometry, "/odom", self.on_odom, 10)

        self.pose = None        # 마커 라이다프레임 (x, y, yaw)
        self.pose_t = 0.0
        self.odom = None        # 로봇 odom (X, Y, yaw)
        self.prev_state = None
        self.success = False
        self.create_timer(1.0 / RATE_HZ, self.control)
        self.get_logger().info(
            f"reflective_dock(디버그) 시작: SNAP→TURN1→DRIVE→TURN2→APPROACH→CREEP "
            f"(목표 라이다 {self.fsm.target_m*100:.1f}cm)"
        )

    # ---- 콜백 ----
    def on_pose(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y
        yaw = 2.0 * math.atan2(msg.pose.orientation.z, msg.pose.orientation.w)
        self.pose = (x, y, yaw)
        self.pose_t = time.monotonic()

    def on_odom(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.odom = (p.x, p.y, 2.0 * math.atan2(q.z, q.w))

    def send(self, vx, wz):
        t = Twist()
        t.linear.x = vx
        t.angular.z = wz
        self.pub.publish(t)

    # ---- FSM 구동(순수 로직에 관측을 먹이고 명령을 낸다) ----
    def control(self):
        now = time.monotonic()
        fresh = self.pose is not None and (now - self.pose_t) <= WATCHDOG_SEC
        v, w = self.fsm.step(self.pose if fresh else None, self.odom, now)
        self.send(v, w)

        if self.fsm.state != self.prev_state:   # 상태 전이 로깅(순수 FSM 은 로그 안 함)
            extra = (f"  (법선까지 {self.fsm.snap_move*100:.1f}cm)"
                     if self.fsm.state in ("TURN1", "TURN2") and self.fsm.snap_move
                     else "")
            self.get_logger().info(f"→ {self.fsm.state}{extra}")
            self.prev_state = self.fsm.state

        if self.fsm.done:
            self.success = (self.fsm.result_code == RC_OK)
            self.get_logger().info(f"종료: {self.fsm.note}")
            for _ in range(10):
                self.send(0.0, 0.0)
                time.sleep(0.02)
            rclpy.shutdown()


def main():
    rclpy.init()
    node = ReflectiveDockNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            for _ in range(10):
                node.pub.publish(Twist())
                time.sleep(0.02)
            rclpy.shutdown()
        ok = node.success
        node.destroy_node()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
