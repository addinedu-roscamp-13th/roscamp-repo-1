#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
charuco_tf_direct — pyrealsense2 SDK로 카메라 직접 읽어 ChArUco 검출→TF 발행
============================================================================
realsense ROS 노드가 자꾸 얼어붙어서(SIGABRT/멈춤), 그걸 우회한다.
pyrealsense2 SDK는 안정적 → 카메라를 직접 읽고, easy_handeye2가 필요한
  camera_color_optical_frame → handeye_target  TF 만 발행한다.
(ROS 카메라 노드/이미지 토픽 불필요. 실시간 검출 이미지는 evidence에 주기 저장.)

보드: DICT_5X5_100, 5x7, square 30mm, marker 23mm  (인쇄한 그 보드)
실행: ROS_DOMAIN_ID=20 python3 charuco_tf_direct.py
"""
import os, time
import numpy as np, cv2
import pyrealsense2 as rs
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Image
from tf2_ros import TransformBroadcaster

SQUARES = (5, 7)
DICT_ID = cv2.aruco.DICT_5X5_100
SQ, MK = 0.030, 0.023
W, H = 1280, 720
OPTICAL_FRAME = "camera_color_optical_frame"
TARGET_FRAME = "handeye_target"
EV = os.path.expanduser("~/Desktop/tomato_pkg_extract/deploy/evidence/2026-07-16_②핸드아이/실시간검출")


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


class CharucoDirect(Node):
    def __init__(self):
        super().__init__("charuco_tf_direct")
        self.dictionary = cv2.aruco.getPredefinedDictionary(DICT_ID)
        self.board = cv2.aruco.CharucoBoard_create(SQUARES[0], SQUARES[1], SQ, MK, self.dictionary)
        try: self.params = cv2.aruco.DetectorParameters_create()
        except AttributeError: self.params = cv2.aruco.DetectorParameters()
        # pyrealsense2 카메라 (SDK 직접)
        self.pipe = rs.pipeline(); cfg = rs.config()
        cfg.enable_stream(rs.stream.color, W, H, rs.format.bgr8, 30)
        prof = self.pipe.start(cfg)
        intr = prof.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        self.K = np.array([[intr.fx,0,intr.ppx],[0,intr.fy,intr.ppy],[0,0,1]], float)
        self.dist = np.array(intr.coeffs, float)
        self.br = TransformBroadcaster(self)
        self.pub_img = self.create_publisher(Image, '/charuco/image_annotated', 3)  # 라이브 뷰
        os.makedirs(EV, exist_ok=True)
        self.n = 0; self.nok = 0; self.last_nc = 0
        self.get_logger().info(f"charuco_tf_direct 시작 — SDK직접 {W}x{H} fx={intr.fx:.1f}")
        self.get_logger().info(f"  발행 TF: {OPTICAL_FRAME} -> {TARGET_FRAME}  (easy_handeye2 tracking용)")
        self.create_timer(0.033, self.tick)   # ~30Hz

    def tick(self):
        try:
            fr = self.pipe.wait_for_frames(2000).get_color_frame()
        except Exception:
            return
        img = np.asanyarray(fr.get_data())
        g = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        mc, mi, _ = cv2.aruco.detectMarkers(g, self.dictionary, parameters=self.params)
        nc = 0; vis = None
        if mi is not None and len(mi) > 0:
            _, cc, ci = cv2.aruco.interpolateCornersCharuco(mc, mi, g, self.board)
            nc = 0 if ci is None else len(ci)
            if nc >= 6:
                ok, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(cc, ci, self.board, self.K, self.dist, None, None)
                if ok:
                    self.publish_tf(rvec, tvec); self.nok += 1
        self.n += 1; self.last_nc = nc
        if self.n % 30 == 0:
            self.get_logger().info(f"  검출 코너:{nc}  {'✅ 보드 잡힘' if nc>=6 else '❌ 보드 안보임'}  (누적성공 {self.nok})")
        # 라이브 뷰: 3프레임마다 오버레이 그려 발행 (+ 90프레임마다 evidence 저장)
        if self.n % 3 == 0:
            vis = img.copy()
            if mi is not None and len(mi) > 0:
                cv2.aruco.drawDetectedMarkers(vis, mc, mi)
                try:
                    _, cc2, ci2 = cv2.aruco.interpolateCornersCharuco(mc, mi, g, self.board)
                    if ci2 is not None: cv2.aruco.drawDetectedCornersCharuco(vis, cc2, ci2, (0,255,0))
                except Exception: pass
            # 화면 중앙 십자선(보드 중앙 맞추기용)
            h, w = vis.shape[:2]
            cv2.drawMarker(vis, (w//2, h//2), (255,255,0), cv2.MARKER_CROSS, 40, 2)
            cv2.putText(vis, f"corners:{nc}/24  {'OK' if nc>=6 else 'NO BOARD'}", (15,35),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0,255,0) if nc>=6 else (0,0,255), 2)
            m = Image(); m.header.stamp = self.get_clock().now().to_msg(); m.header.frame_id = OPTICAL_FRAME
            m.height, m.width = vis.shape[:2]; m.encoding = 'bgr8'; m.is_bigendian = 0
            m.step = m.width*3; m.data = vis.tobytes()
            self.pub_img.publish(m)
            if self.n % 90 == 0 and nc > 0:
                self.save_i = getattr(self, 'save_i', 0) + 1
                cv2.imwrite(os.path.join(EV, f"detect_{self.save_i:04d}_c{nc}.jpg"), vis)
                cv2.imwrite(os.path.join(EV, "latest.jpg"), vis)

    def publish_tf(self, rvec, tvec):
        R, _ = cv2.Rodrigues(rvec); q = mat_to_quat(R)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = OPTICAL_FRAME; t.child_frame_id = TARGET_FRAME
        t.transform.translation.x = float(tvec[0]); t.transform.translation.y = float(tvec[1]); t.transform.translation.z = float(tvec[2])
        t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = q
        self.br.sendTransform(t)


def main():
    rclpy.init(); n = CharucoDirect()
    try: rclpy.spin(n)
    except KeyboardInterrupt: pass
    finally:
        try: n.pipe.stop()
        except Exception: pass
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
