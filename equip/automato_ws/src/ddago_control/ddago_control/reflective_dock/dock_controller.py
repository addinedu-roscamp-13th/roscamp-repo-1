#!/usr/bin/env python3
"""
반사테이프 후진 도킹 — 단일 FSM 노드 (사전정렬 + 후진 통합).

prealign_normal.py(법선 사전정렬)와 reverse_dock_diag.py(법선교정 후진)를
'한 노드의 상태들'로 이어붙였다. 명령 하나로 도킹 전체가 돈다.

상태 흐름:
  SNAP     멈춘 채 마커 pose 평균 → 법선(odom) 스냅, 법선 위 수선의 발 P 계산
  TURN1    제자리 회전으로 P 바라봄
  DRIVE    P까지 직진(옆으로 벗어난 만큼)
  TURN2    뒤축이 마커를 향하도록(법선) 회전            ── 여기까지 사전정렬
  APPROACH 후진하며 β(중앙)·e(법선이탈) 동시 교정
  CREEP    마지막 블라인드 후진(odom 거리)              ── 여기부터 후진
  → 성공 종료(exit 0)

핵심 변경: 예전엔 'prealign 성공(exit0) → 쉘이 reverse 실행'(gating)이었다.
  이제 TURN2 완료 시 종료 대신 state=APPROACH 로 넘겨 내부에서 이어진다.
  실패는 각 단계 안전조건에서 정지+exit 1.
  watchdog(마커 신선도)은 APPROACH 에서만(회전/직진은 odom이라 마커 없어도 됨).
  TURN2→APPROACH 전환 순간 마커가 아직 뒤 시야에 안 들어왔을 수 있어,
  '한 번이라도 좋은 값을 본 뒤(last_good_t)'부터 상실 판정 → 재획득 유예를 준다.

실행: python3 reflective_dock.py   (다른 로봇: -p rear_offset_m:=0.055)
"""
import math
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry

# 라이다 오프셋(detector_node.to_odom 와 동일, URDF 실측)
LIDAR_X = -0.017
LIDAR_Y = 0.0
LIDAR_YAW_OFFSET = math.pi

# ── 사전정렬(SNAP~TURN2) 튜닝 ──
SNAP_FRAMES = 15
KW = 1.2            # 회전 비례게인
KV = 0.4            # 직진 비례게인
TURN_TOL_DEG = 3.0
DRIVE_TOL_M = 0.01
MIN_MOVE_M = 0.02   # 이보다 적게 벗어났으면 DRIVE 생략
MAX_MOVE_M = 0.30   # 이보다 크게 나오면 스냅샷 이상 → 안전 중단
WMAX = 0.4          # 회전 상한(사전정렬)
WMIN = 0.15         # 데드밴드 극복 최소 회전

# ── 후진(APPROACH~CREEP) 튜닝 ──
VMAX = 0.05
VMIN = 0.015
KDIST = 0.4
KBETA = 0.8         # β(중앙 보기) 게인
KE = 0.7            # e(법선 끌어당김) 게인
SIGN_E = +1.0       # e 교정 방향(실측 +1)
MAX_BETA_DEG = 60.0 # |β| 과대면 오검출로 무시
WCAP = 0.3          # 회전 상한(후진)
SWITCH_M = 0.18     # 이 거리 이하면 크립 전환
CREEP_V = 0.015
CREEP_MAX = 0.09
DMAX_MARGIN = 0.03

# ── 공통 ──
WATCHDOG_SEC = 0.5
MAX_SEC = 60.0      # 도킹 전체 제한시간
RATE_HZ = 20


def norm_ang(a):
    """각도를 -pi~pi 로 정규화."""
    return math.atan2(math.sin(a), math.cos(a))


class ReflectiveDock(Node):
    def __init__(self):
        super().__init__("reflective_dock")
        self.declare_parameter("rear_offset_m", 0.10)
        self.declare_parameter("stop_gap_m", 0.02)
        rear = self.get_parameter("rear_offset_m").value
        gap = self.get_parameter("stop_gap_m").value
        self.target_m = rear + gap      # 정지 목표(라이다 거리)
        self.d_emergency = rear + 0.005  # 비상정지 거리

        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(PoseStamped, "/docking_marker_pose", self.on_pose, 10)
        self.create_subscription(Odometry, "/odom", self.on_odom, 10)

        self.pose = None        # 마커 라이다프레임 (x, y, yaw)
        self.pose_t = 0.0
        self.odom = None        # 로봇 odom (X, Y, yaw)
        self.state = "SNAP"
        # 사전정렬용
        self.snap = []
        self.P = None
        self.theta_final = None
        self.heading_to_P = None
        # 후진용
        self.d0 = None
        self.last_good_t = None
        self.creep_origin = None
        self.creep_dist = 0.0
        self.success = False
        self.n = 0
        self.start_t = time.monotonic()
        self.create_timer(1.0 / RATE_HZ, self.control)
        self.get_logger().info(
            f"reflective_dock 시작: SNAP→TURN1→DRIVE→TURN2→APPROACH→CREEP "
            f"(목표 라이다 {self.target_m*100:.1f}cm)"
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

    def to_odom(self, x_l, y_l):
        """라이다 프레임 점 → odom 좌표. 반환 (ox, oy, a). a=라이다헤딩(odom)."""
        X, Y, th = self.odom
        lox = X + LIDAR_X * math.cos(th) - LIDAR_Y * math.sin(th)
        loy = Y + LIDAR_X * math.sin(th) + LIDAR_Y * math.cos(th)
        a = th + LIDAR_YAW_OFFSET
        ox = lox + x_l * math.cos(a) - y_l * math.sin(a)
        oy = loy + x_l * math.sin(a) + y_l * math.cos(a)
        return ox, oy, a

    # ---- 헬퍼 ----
    def stop(self):
        self.pub.publish(Twist())

    def send(self, vx, wz):
        t = Twist()
        t.linear.x = vx
        t.angular.z = wz
        self.pub.publish(t)

    def finish(self, reason, success=False):
        self.success = success
        self.get_logger().info(f"종료: {reason}")
        for _ in range(10):
            self.stop()
            time.sleep(0.02)
        rclpy.shutdown()

    def fresh(self, now):
        return self.pose is not None and (now - self.pose_t) <= WATCHDOG_SEC

    # ---- 상태기계 ----
    def control(self):
        now = time.monotonic()
        if now - self.start_t > MAX_SEC:
            self.finish("전체 제한시간 초과")
            return
        if self.odom is None:
            return  # odom 대기
        s = self.state
        if s == "SNAP":
            self.do_snap(now)
        elif s == "TURN1":
            self.do_turn(self.heading_to_P, "DRIVE")
        elif s == "DRIVE":
            self.do_drive()
        elif s == "TURN2":
            self.do_turn(self.theta_final, "APPROACH")  # ★ 여기서 후진으로 이어짐
        elif s == "APPROACH":
            self.do_approach(now)
        elif s == "CREEP":
            self.do_creep()

    # ---- SNAP~TURN2 (사전정렬) ----
    def do_snap(self, now):
        self.stop()  # 스냅샷 동안 정지
        if not self.fresh(now):
            return
        x, y, yaw_l = self.pose
        ox, oy, a = self.to_odom(x, y)
        n_odom = yaw_l + a  # 라이다프레임 법선 → odom 방향
        self.snap.append((ox, oy, math.cos(n_odom), math.sin(n_odom)))
        if len(self.snap) < SNAP_FRAMES:
            return
        k = len(self.snap)
        Mx = sum(s[0] for s in self.snap) / k
        My = sum(s[1] for s in self.snap) / k
        n = math.atan2(sum(s[3] for s in self.snap), sum(s[2] for s in self.snap))
        self.compute_goal(Mx, My, n)

    def compute_goal(self, Mx, My, n):
        """마커 꼭짓점 M·법선 n → 수선의 발 P·최종 헤딩."""
        Rx, Ry, _ = self.odom
        ux, uy = math.cos(n), math.sin(n)
        proj = (Rx - Mx) * ux + (Ry - My) * uy
        Px, Py = Mx + proj * ux, My + proj * uy
        move = math.hypot(Px - Rx, Py - Ry)
        self.P = (Px, Py)
        self.theta_final = n
        self.heading_to_P = math.atan2(Py - Ry, Px - Rx)
        self.get_logger().info(
            f"스냅샷: 마커 odom=({Mx:+.2f},{My:+.2f}) 법선={math.degrees(n):+.0f}° "
            f"→ 법선까지 {move*100:.1f}cm"
        )
        if move > MAX_MOVE_M:
            self.finish(f"이동거리 {move*100:.0f}cm 과대 — 스냅샷 이상, 중단")
            return
        if move < MIN_MOVE_M:
            self.get_logger().info("이미 법선 위 → DRIVE 생략")
            self.state = "TURN2"
        else:
            self.state = "TURN1"

    def do_turn(self, target_th, next_state):
        _, _, Rth = self.odom
        err = norm_ang(target_th - Rth)
        if abs(err) < math.radians(TURN_TOL_DEG):
            self.stop()
            self.get_logger().info(
                f"{self.state} 완료(err={math.degrees(err):+.1f}°) → {next_state}"
            )
            self.state = next_state
            return
        wz = max(-WMAX, min(WMAX, KW * err))
        if abs(wz) < WMIN:
            wz = math.copysign(WMIN, err)
        self.send(0.0, wz)
        self.n += 1
        if self.n % 10 == 0:
            self.get_logger().info(f"{self.state} err={math.degrees(err):+.1f}°")

    def do_drive(self):
        Rx, Ry, Rth = self.odom
        rem = math.hypot(self.P[0] - Rx, self.P[1] - Ry)
        head = math.atan2(self.P[1] - Ry, self.P[0] - Rx)
        err = norm_ang(head - Rth)
        if rem < DRIVE_TOL_M or abs(err) > math.radians(90.0):
            self.stop()
            self.get_logger().info(f"DRIVE 완료(남음 {rem*100:.1f}cm) → TURN2")
            self.state = "TURN2"
            return
        v = max(VMIN, min(VMAX, KV * rem))
        wz = max(-WMAX, min(WMAX, KW * err))
        self.send(v, wz)
        self.n += 1
        if self.n % 10 == 0:
            self.get_logger().info(
                f"DRIVE 남음 {rem*100:.1f}cm err={math.degrees(err):+.1f}°"
            )

    # ---- APPROACH~CREEP (후진) ----
    def do_approach(self, now):
        fresh = self.fresh(now)
        if not fresh:
            self.stop()
            # 한 번이라도 좋은 값을 본 뒤에만 상실 판정(TURN2 직후 재획득 유예).
            if self.last_good_t is not None and (now - self.last_good_t) > 0.7:
                self.finish("마커 상실 — 정지")
            return

        x, y, yaw = self.pose
        d = math.hypot(x, y)
        beta = math.atan2(y, x)
        e = -x * math.sin(yaw) + y * math.cos(yaw)  # 법선축 이탈(cross-track)
        tilt = abs(((math.degrees(yaw) - 180.0 + 180.0) % 360.0) - 180.0)

        if abs(beta) > math.radians(MAX_BETA_DEG):  # 스퓨리어스 차단
            self.stop()
            if self.last_good_t is not None and (now - self.last_good_t) > 1.0:
                self.finish("마커 이상(β 과대) — 정지")
            return
        self.last_good_t = now

        if self.d0 is None:
            self.d0 = d
            self.get_logger().info(
                f"APPROACH 시작 d={d*100:.1f}cm β={math.degrees(beta):+.1f}° "
                f"e={e*100:+.1f}cm tilt={tilt:.1f}°"
            )
        if d < self.d_emergency:
            self.finish(f"비상정지 d={d*100:.1f}cm")
            return
        if d > self.d0 + DMAX_MARGIN:
            self.finish(f"거리 증가 d={d*100:.1f}cm — 방향 이상")
            return
        if d <= SWITCH_M:
            self.creep_origin = (self.odom[0], self.odom[1])
            self.creep_dist = max(0.0, min(CREEP_MAX, d - self.target_m))
            self.state = "CREEP"
            self.get_logger().info(
                f"전환: d={d*100:.1f}cm e={e*100:+.1f}cm tilt={tilt:.1f}° "
                f"→ 크립 {self.creep_dist*100:.1f}cm"
            )
            return

        v = max(VMIN, min(VMAX, KDIST * (d - self.target_m)))
        wz = max(-WCAP, min(WCAP, KBETA * beta + SIGN_E * KE * e))
        self.send(-v, wz)
        self.n += 1
        if self.n % 10 == 0:
            self.get_logger().info(
                f"APPROACH d={d*100:.1f}cm β={math.degrees(beta):+.1f}° "
                f"e={e*100:+.1f}cm tilt={tilt:.1f}°"
            )

    def do_creep(self):
        if self.creep_origin is None:
            self.stop()
            return
        traveled = math.hypot(
            self.odom[0] - self.creep_origin[0], self.odom[1] - self.creep_origin[1]
        )
        if traveled >= self.creep_dist:
            gap = self.get_parameter("stop_gap_m").value
            self.finish(
                f"도킹 완료(크립 {traveled*100:.1f}cm) → 라이다≈{self.target_m*100:.1f}cm, "
                f"뒤 끝 여유≈{gap*100:.0f}cm",
                success=True,
            )
            return
        self.send(-CREEP_V, 0.0)
        self.n += 1
        if self.n % 10 == 0:
            self.get_logger().info(f"CREEP {traveled*100:.1f}/{self.creep_dist*100:.1f}cm")


def main():
    rclpy.init()
    node = ReflectiveDock()
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
