#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
charuco_tf_publisher — ChArUco 보드를 카메라로 검출해 TF로 발행
================================================================
easy_handeye2 는 순수 TF 프레임만으로 핸드아이를 푼다(move_group 불필요).
그래서 "카메라 → 보드(타겟)" 변환을 TF로 공급해줄 노드가 필요하다.
이 노드가 그 역할:
  /camera/camera/color/image_raw (+ camera_info) 를 구독
  → ChArUco 보드 검출 → solvePnP → TF 발행:
      camera_color_optical_frame → handeye_target

easy_handeye2 설정(카메라↔베이스, eye-in-hand):
  robot_base_frame     = base_link              (arm_tf_bridge.py)
  robot_effector_frame = gripper_link           (arm_tf_bridge.py)
  tracking_base_frame  = camera_color_optical_frame
  tracking_marker_frame= handeye_target         (이 노드)

보드: DICT_5X5_100, 5x7, square 30mm, marker 23mm  (네 실제 보드)
실행: source ROS + handeye_ws; python3 charuco_tf_publisher.py
"""
import numpy as np, cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

SQUARES = (5, 7)
DICT_ID = cv2.aruco.DICT_5X5_100
SQ, MK = 0.030, 0.023                    # m
IMAGE_TOPIC = "/camera/camera/color/image_raw"
INFO_TOPIC  = "/camera/camera/color/camera_info"
OPTICAL_FRAME = "camera_color_optical_frame"
TARGET_FRAME  = "handeye_target"

def img_to_bgr(msg: Image):
    """cv_bridge 없이 sensor_msgs/Image → BGR numpy"""
    h, w = msg.height, msg.width
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    enc = msg.encoding.lower()
    if enc in ("rgb8", "bgr8"):
        img = buf.reshape(h, w, 3)
        return cv2.cvtColor(img, cv2.COLOR_RGB2BGR) if enc == "rgb8" else img
    if enc in ("mono8",):
        return cv2.cvtColor(buf.reshape(h, w), cv2.COLOR_GRAY2BGR)
    # 기타(예: rgba8)
    img = buf.reshape(h, w, -1)[:, :, :3]
    return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

class CharucoTF(Node):
    def __init__(self):
        super().__init__("charuco_tf_publisher")
        # OpenCV 4.6 레거시 API (시스템 python3). CharucoBoard_create(squaresX, squaresY, ...)
        self.dictionary = cv2.aruco.getPredefinedDictionary(DICT_ID)
        self.board = cv2.aruco.CharucoBoard_create(SQUARES[0], SQUARES[1], SQ, MK, self.dictionary)
        try: self.aruco_params = cv2.aruco.DetectorParameters_create()
        except AttributeError: self.aruco_params = cv2.aruco.DetectorParameters()
        self.K = None; self.dist = None
        self.br = TransformBroadcaster(self)
        self.create_subscription(CameraInfo, INFO_TOPIC, self.on_info, 10)
        self.create_subscription(Image, IMAGE_TOPIC, self.on_img, 10)
        self.n_ok = 0; self.n_frame = 0
        self.get_logger().info(f"charuco_tf_publisher 시작 — 보드 {SQUARES} DICT_5X5_100 sq{SQ*1000:.0f} mk{MK*1000:.0f}")
        self.get_logger().info(f"  구독:{IMAGE_TOPIC}  발행 TF:{OPTICAL_FRAME}->{TARGET_FRAME}")

    def on_info(self, msg: CameraInfo):
        if self.K is None:
            self.K = np.array(msg.k, float).reshape(3, 3)
            self.dist = np.array(msg.d, float) if len(msg.d) else np.zeros(5)
            self.get_logger().info(f"  카메라 내부값 수신: fx={self.K[0,0]:.1f} fy={self.K[1,1]:.1f}")

    def on_img(self, msg: Image):
        if self.K is None:
            return
        self.n_frame += 1
        img = img_to_bgr(msg)
        g = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        # 레거시 검출: 마커 → charuco 코너 보간 → 보드 자세
        mc, mi, _ = cv2.aruco.detectMarkers(g, self.dictionary, parameters=self.aruco_params)
        nc = 0
        if mi is not None and len(mi) > 0:
            _, cc, ci = cv2.aruco.interpolateCornersCharuco(mc, mi, g, self.board)
            nc = 0 if ci is None else len(ci)
            if nc >= 6:
                ok, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(
                    cc, ci, self.board, self.K, self.dist, None, None)
                if ok:
                    self.publish_tf(rvec, tvec, msg.header.stamp)
                    self.n_ok += 1
        # 1초에 한 번쯤 상태 로그
        if self.n_frame % 30 == 0:
            self.get_logger().info(f"  검출 코너:{nc}  (누적 성공 {self.n_ok}프레임)  "
                                   f"{'✅ 보드 잡힘' if nc>=6 else '❌ 보드 안보임'}")

    def publish_tf(self, rvec, tvec, stamp):
        R, _ = cv2.Rodrigues(rvec)
        q = self.mat_to_quat(R)
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = OPTICAL_FRAME
        t.child_frame_id = TARGET_FRAME
        t.transform.translation.x = float(tvec[0]); t.transform.translation.y = float(tvec[1]); t.transform.translation.z = float(tvec[2])
        t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = q
        self.br.sendTransform(t)

    @staticmethod
    def mat_to_quat(R):
        tr = R[0,0]+R[1,1]+R[2,2]
        if tr > 0:
            s = np.sqrt(tr+1.0)*2; qw=0.25*s
            qx=(R[2,1]-R[1,2])/s; qy=(R[0,2]-R[2,0])/s; qz=(R[1,0]-R[0,1])/s
        elif R[0,0]>R[1,1] and R[0,0]>R[2,2]:
            s=np.sqrt(1.0+R[0,0]-R[1,1]-R[2,2])*2; qw=(R[2,1]-R[1,2])/s
            qx=0.25*s; qy=(R[0,1]+R[1,0])/s; qz=(R[0,2]+R[2,0])/s
        elif R[1,1]>R[2,2]:
            s=np.sqrt(1.0+R[1,1]-R[0,0]-R[2,2])*2; qw=(R[0,2]-R[2,0])/s
            qx=(R[0,1]+R[1,0])/s; qy=0.25*s; qz=(R[1,2]+R[2,1])/s
        else:
            s=np.sqrt(1.0+R[2,2]-R[0,0]-R[1,1])*2; qw=(R[1,0]-R[0,1])/s
            qx=(R[0,2]+R[2,0])/s; qy=(R[1,2]+R[2,1])/s; qz=0.25*s
        return (float(qx),float(qy),float(qz),float(qw))

def main():
    rclpy.init(); n = CharucoTF()
    try: rclpy.spin(n)
    except KeyboardInterrupt: pass
    finally:
        rclpy.try_shutdown()

if __name__ == "__main__":
    main()
