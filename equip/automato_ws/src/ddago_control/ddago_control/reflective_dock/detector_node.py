#!/usr/bin/env python3
"""
반사마커 검출 ROS2 노드.

/scan (LaserScan) 구독 → 매 프레임 검출 파이프라인 실행 →
  /docking_markers      (MarkerArray)  : RViz 시각화 (면 선·코너·화살표·코드)
  /docking_marker_pose  (PoseStamped)  : 검출된 마커의 위치·방향
발행. 단계별 개수는 로그(DEBUG), 마커 검출 시 INFO 로그.

실행:  python3 detector_node.py
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray

from .marker_detector import run_pipeline

# 기본 파라미터 (오프라인 테스트로 정한 값, 런타임 조정 가능)
CFG = {
    "r_min": 0.15,
    "r_max": 0.55,
    "gap": 0.03,
    "iepf": 0.02,
    "seg_len": 0.13,   # 면 길이 중심(m)
    "seg_tol": 0.06,   # 허용폭 → 통과 7~19cm. 7cm 마커가 각도·blooming으로
                       # 9~15cm로 들쭉날쭉해 넓게 잡아야 놓치지 않음(파편 3.8cm는 탈락)
    "min_points": 10,
    "angle_tol": 15.0,
    "endpoint_tol": 0.05,
    "reflective_min": 50.0,
    "n_bits": 3,
}

SPIKE_SKIP_MAX = 5  # 시간평활: 연속 이만큼 스킵되면 기준 재설정(영구 잠김 방지)
LOCK_ACQUIRE_FRAMES = 5  # lock 획득: 정면 후보가 연속 이만큼 안정돼야 확정(첫프레임 오lock 방지)
ACQ_MATCH_M = 0.05       # 획득 중 '같은 후보' 판정 거리(이내면 연속으로 카운트)


class MarkerDetector(Node):
    def __init__(self):
        super().__init__("marker_detector")
        for k, v in CFG.items():
            self.declare_parameter(k, v)
        # LaserScan 은 보통 BEST_EFFORT 로 발행 → sensor_data QoS 로 구독
        self.create_subscription(
            LaserScan, "/scan", self.on_scan, qos_profile_sensor_data
        )
        self.pub_markers = self.create_publisher(MarkerArray, "/docking_markers", 10)
        self.pub_pose = self.create_publisher(PoseStamped, "/docking_marker_pose", 10)

        # --- Step 2: 월드(odom) 좌표 lock 추적 ---
        # base_footprint→rplidar_link 오프셋(URDF 실측): x=-0.017m, y=0, yaw=π.
        # 라이다가 회전중심 뒤 1.7cm에 있고 180° 뒤를 봄. 회전 시 궤도(작음) 보정용.
        self.declare_parameter("lidar_yaw_offset_rad", math.pi)
        self.declare_parameter("lidar_x_m", -0.017)  # 회전중심→라이다 x(base 프레임)
        self.declare_parameter("lidar_y_m", 0.0)     # 회전중심→라이다 y(base 프레임)
        self.declare_parameter("lock_gate_m", 0.12)  # lock에서 이보다 멀면 목표 아님(옆 마커 차단 여유마진)
        self.declare_parameter("spike_jump_m", 0.07)  # 프레임간 월드 점프 이보다 크면 스파이크로 스킵
        self.create_subscription(Odometry, "/odom", self.on_odom, 10)
        self.odom = None   # (X, Y, yaw) 로봇 pose(odom 프레임)
        self.lock = None   # (ox, oy) 목표 마커의 odom 좌표(고정)
        self.last_pub_world = None  # 직전 발행 마커의 월드좌표(스파이크 판정 기준)
        self.skip_count = 0         # 연속 스파이크 스킵 횟수
        self.acq_world = None       # lock 획득 중 후보 월드좌표
        self.acq_count = 0          # 후보가 연속 안정된 프레임 수

        self.get_logger().info("marker_detector 시작. /scan·/odom 구독 중...")

    def _cfg(self):
        return {k: self.get_parameter(k).value for k in CFG}

    def on_odom(self, msg):
        """로봇의 odom pose (X, Y, yaw) 저장. 라이다→월드 변환에 씀."""
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = 2.0 * math.atan2(q.z, q.w)  # 순수 z 쿼터니언 → yaw
        self.odom = (p.x, p.y, yaw)

    def to_odom(self, x_l, y_l):
        """라이다 프레임 점 (x_l,y_l) → 월드(odom) 좌표. odom 없으면 None.

        2단계 변환:
          ① 라이다 위치(월드) = 회전중심 + base 회전으로 (lx,ly) 오프셋을 돌린 것
             → 로봇이 돌 때 라이다가 그리는 궤도를 반영.
          ② 마커(월드) = 라이다 위치 + 라이다 헤딩(=로봇 헤딩+yaw offset)으로
             마커의 라이다상대위치를 돌린 것.
        """
        if self.odom is None:
            return None
        X, Y, th = self.odom
        lx = self.get_parameter("lidar_x_m").value
        ly = self.get_parameter("lidar_y_m").value
        # ① 라이다의 월드 위치(회전중심에서 오프셋만큼, base 방향으로 회전)
        lox = X + lx * math.cos(th) - ly * math.sin(th)
        loy = Y + lx * math.sin(th) + ly * math.cos(th)
        # ② 마커의 월드 위치(라이다 위치 기준, 라이다 헤딩으로 상대위치 회전)
        a = th + self.get_parameter("lidar_yaw_offset_rad").value
        ox = lox + x_l * math.cos(a) - y_l * math.sin(a)
        oy = loy + x_l * math.sin(a) + y_l * math.cos(a)
        return (ox, oy)

    def select_target(self, markers):
        """여러 마커 중 '도킹 목표' 하나를 고른다 (공간 게이팅 Step 1+2).

        - 처음(lock 없음): 정면(|y| 최소)으로 고르고 그 월드(odom) 좌표를 lock.
          nav2 가 로봇을 목표 정면에 세우므로 |y| 최소 = 내 목표(밝기 무관).
        - 이후: lock 한 월드 위치에 가장 가까운 마커를 채택(회전해도 월드 위치는 고정).
          가장 가까운 것도 게이트(lock_gate_m)보다 멀면 목표를 못 본 것 → None.
        - odom 아직 없으면 Step 1(|y| 최소)로 임시 동작.
        """
        if self.odom is None:  # odom 도착 전: Step 1 임시
            return min(markers, key=lambda m: abs(m["y"]))

        if self.lock is None:  # 최초 획득: 정면(min|y|)이 연속 N프레임 안정되면 확정
            m0 = min(markers, key=lambda m: abs(m["y"]))
            o0 = self.to_odom(m0["x"], m0["y"])
            if self.acq_world is not None and math.hypot(
                o0[0] - self.acq_world[0], o0[1] - self.acq_world[1]) <= ACQ_MATCH_M:
                self.acq_count += 1          # 같은 자리 → 연속 카운트
            else:
                self.acq_world = o0          # 후보 바뀜(목표 잠깐 빠져 옆 게 정면) → 리셋
                self.acq_count = 1
            if self.acq_count >= LOCK_ACQUIRE_FRAMES:
                self.lock = self.acq_world
                self.last_pub_world = self.lock  # 스파이크 판정 기준 초기화
                self.get_logger().info(
                    f"목표 lock(안정 {self.acq_count}프레임): "
                    f"월드=({self.lock[0]:+.2f},{self.lock[1]:+.2f})"
                )
            return m0  # 획득 중에도 정면 후보는 잠정 발행

        # lock 이후: 월드에서 lock에 가장 가까운 마커
        gate = self.get_parameter("lock_gate_m").value
        best, best_o, best_d = None, None, 1e9
        for m in markers:
            o = self.to_odom(m["x"], m["y"])
            d = math.hypot(o[0] - self.lock[0], o[1] - self.lock[1])
            if d < best_d:
                best, best_o, best_d = m, o, d
        if best_d > gate:
            return None  # 게이트 밖(옆 마커) → 채택 안 함

        # 시간 평활: 정지 마커의 월드좌표가 한 프레임에 확 튀면 스파이크로 보고 스킵.
        # (연속 SPIKE_SKIP_MAX 회 튀면 진짜 이동으로 보고 기준 재설정 → 영구 잠김 방지)
        jump = math.hypot(best_o[0] - self.last_pub_world[0],
                          best_o[1] - self.last_pub_world[1])
        if jump > self.get_parameter("spike_jump_m").value and self.skip_count < SPIKE_SKIP_MAX:
            self.skip_count += 1
            return None  # 이 프레임은 튐 → 발행 안 함
        self.skip_count = 0
        self.last_pub_world = best_o
        return best

    def on_scan(self, msg):
        # LaserScan → (각도[도], 거리, 밝기) 목록
        readings = []
        n = len(msg.ranges)
        has_inten = len(msg.intensities) == n
        for i in range(n):
            r = msg.ranges[i]
            if not math.isfinite(r) or r <= 0.0:
                continue
            deg = math.degrees(msg.angle_min + i * msg.angle_increment)
            inten = msg.intensities[i] if has_inten else 0.0
            readings.append((deg, r, inten))

        try:
            markers, counts = run_pipeline(readings, self._cfg())
        except Exception as e:  # 한 프레임 오류로 노드가 죽지 않게
            self.get_logger().warn(f"파이프라인 오류: {e}")
            return

        self.get_logger().debug(
            f"raw{counts['raw']} range{counts['range']} clu{counts['clusters']} "
            f"seg{counts['segments']} len{counts['len_filtered']} "
            f"pair{counts['pairs']} marker{counts['markers']}"
        )

        frame = msg.header.frame_id or "rplidar_link"
        target = self.select_target(markers) if markers else None
        self.publish_markers(markers, target, frame, msg.header.stamp)
        if target is not None:
            o = self.to_odom(target["x"], target["y"])
            if o is not None and self.lock is not None:
                dlock = math.hypot(o[0] - self.lock[0], o[1] - self.lock[1])
                self.get_logger().info(
                    f"마커 {len(markers)}개 → 채택 라이다(x={target['x']:+.2f} y={target['y']:+.2f}) "
                    f"월드=({o[0]:+.2f},{o[1]:+.2f}) lock거리={dlock*100:.1f}cm"
                )
            self.publish_pose(target, frame, msg.header.stamp)
        elif markers:  # 마커는 보이나 목표(lock)로부터 멀어 채택 안 함
            self.get_logger().info(f"마커 {len(markers)}개 보이나 lock 밖 → 발행 안 함")

    def publish_pose(self, m, frame, stamp):
        ps = PoseStamped()
        ps.header.frame_id = frame
        ps.header.stamp = stamp
        ps.pose.position.x = float(m["x"])
        ps.pose.position.y = float(m["y"])
        ps.pose.orientation.z = math.sin(m["yaw_rad"] / 2.0)
        ps.pose.orientation.w = math.cos(m["yaw_rad"] / 2.0)
        self.pub_pose.publish(ps)

    # ---- RViz 시각화 ----
    def publish_markers(self, markers, target, frame, stamp):
        arr = MarkerArray()
        clear = Marker()
        clear.header.frame_id = frame
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)  # 매 프레임 이전 것 지우고 새로 그림

        mid = 0
        for m in markers:
            is_target = (m is target)  # 채택된 마커면 강조해서 그림
            arr.markers.append(self._line(m["face_a"], frame, stamp, mid, (0.1, 0.9, 0.3)))
            mid += 1
            arr.markers.append(self._line(m["face_b"], frame, stamp, mid, (0.2, 0.5, 1.0)))
            mid += 1
            arr.markers.append(self._sphere(m["x"], m["y"], frame, stamp, mid, is_target))
            mid += 1
            arr.markers.append(self._arrow(m, frame, stamp, mid))
            mid += 1
            arr.markers.append(self._text(m, frame, stamp, mid, is_target))
            mid += 1
        self.pub_markers.publish(arr)

    def _base(self, frame, stamp, mid):
        mk = Marker()
        mk.header.frame_id = frame
        mk.header.stamp = stamp
        mk.ns = "docking"
        mk.id = mid
        mk.action = Marker.ADD
        mk.pose.orientation.w = 1.0
        return mk

    def _line(self, seg, frame, stamp, mid, rgb):
        mk = self._base(frame, stamp, mid)
        mk.type = Marker.LINE_STRIP
        mk.scale.x = 0.008
        mk.color.r, mk.color.g, mk.color.b, mk.color.a = rgb[0], rgb[1], rgb[2], 1.0
        mk.points = [Point(x=float(p.x), y=float(p.y), z=0.0) for p in seg]
        return mk

    def _sphere(self, x, y, frame, stamp, mid, is_target=False):
        mk = self._base(frame, stamp, mid)
        mk.type = Marker.SPHERE
        mk.pose.position.x = float(x)
        mk.pose.position.y = float(y)
        if is_target:  # 채택 = 초록 크게
            mk.scale.x = mk.scale.y = mk.scale.z = 0.05
            mk.color.r, mk.color.g, mk.color.b, mk.color.a = 0.1, 1.0, 0.2, 1.0
        else:          # 그 외 = 회색 작게
            mk.scale.x = mk.scale.y = mk.scale.z = 0.03
            mk.color.r, mk.color.g, mk.color.b, mk.color.a = 0.5, 0.5, 0.5, 1.0
        return mk

    def _arrow(self, m, frame, stamp, mid):
        mk = self._base(frame, stamp, mid)
        mk.type = Marker.ARROW
        mk.pose.position.x = float(m["x"])
        mk.pose.position.y = float(m["y"])
        mk.pose.orientation.z = math.sin(m["yaw_rad"] / 2.0)
        mk.pose.orientation.w = math.cos(m["yaw_rad"] / 2.0)
        mk.scale.x, mk.scale.y, mk.scale.z = 0.12, 0.02, 0.02
        mk.color.r, mk.color.g, mk.color.b, mk.color.a = 1.0, 0.2, 0.6, 1.0
        return mk

    def _text(self, m, frame, stamp, mid, is_target=False):
        mk = self._base(frame, stamp, mid)
        mk.type = Marker.TEXT_VIEW_FACING
        mk.pose.position.x = float(m["x"])
        mk.pose.position.y = float(m["y"])
        mk.pose.position.z = 0.06
        mk.scale.z = 0.04
        mk.color.r = mk.color.g = mk.color.b = mk.color.a = 1.0
        tag = " <=채택" if is_target else ""
        mk.text = f"|y|{abs(m['y'])*100:.0f}cm{tag}"
        return mk


def main():
    rclpy.init()
    node = MarkerDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
