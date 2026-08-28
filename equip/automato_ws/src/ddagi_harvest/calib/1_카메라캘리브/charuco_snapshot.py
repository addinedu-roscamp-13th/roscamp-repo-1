#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
charuco_snapshot — ROS 카메라 토픽에서 한 프레임 받아 ChArUco 검출·저장
자세 잡는 중 "지금 보드가 보이나?"를 매번 확인하는 용도.
출력: 검출 코너 수 + 저장경로. 코너 그려진 이미지를 evidence에 저장.
사용: python3 charuco_snapshot.py [저장이름]
"""
import sys, os, time
import numpy as np, cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo

SQUARES=(5,7); DICT_ID=cv2.aruco.DICT_5X5_100; SQ,MK=0.030,0.023
IMG="/camera/camera/color/image_raw"; INFO="/camera/camera/color/camera_info"
EV=os.path.expanduser("~/Desktop/tomato_pkg_extract/deploy/evidence/2026-07-16_②핸드아이/자세잡기")

def img_to_bgr(msg):
    h,w=msg.height,msg.width; buf=np.frombuffer(msg.data,np.uint8); enc=msg.encoding.lower()
    if enc in("rgb8","bgr8"):
        img=buf.reshape(h,w,3); return cv2.cvtColor(img,cv2.COLOR_RGB2BGR) if enc=="rgb8" else img
    if enc=="mono8": return cv2.cvtColor(buf.reshape(h,w),cv2.COLOR_GRAY2BGR)
    return cv2.cvtColor(buf.reshape(h,w,-1)[:,:,:3],cv2.COLOR_RGB2BGR)

class Snap(Node):
    def __init__(self,name):
        super().__init__("charuco_snapshot")
        self.name=name; self.img=None; self.K=None; self.dist=None
        self.dictionary=cv2.aruco.getPredefinedDictionary(DICT_ID)
        self.board=cv2.aruco.CharucoBoard_create(SQUARES[0],SQUARES[1],SQ,MK,self.dictionary)
        try: self.params=cv2.aruco.DetectorParameters_create()
        except AttributeError: self.params=cv2.aruco.DetectorParameters()
        self.create_subscription(CameraInfo,INFO,self.on_info,10)
        self.create_subscription(Image,IMG,self.on_img,10)
    def on_info(self,m):
        if self.K is None:
            self.K=np.array(m.k,float).reshape(3,3); self.dist=np.array(m.d,float) if len(m.d) else np.zeros(5)
    def on_img(self,m):
        if self.img is None: self.img=m

def main():
    name=sys.argv[1] if len(sys.argv)>1 else time.strftime("%H%M%S")
    os.makedirs(EV,exist_ok=True)
    rclpy.init(); n=Snap(name)
    t0=time.time()
    while (n.img is None or n.K is None) and time.time()-t0<5:
        rclpy.spin_once(n,timeout_sec=0.1)
    if n.img is None or n.K is None:
        print("RESULT corners=-1 (카메라 프레임/내부값 못받음)"); rclpy.try_shutdown(); return
    img=img_to_bgr(n.img); g=cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    mc,mi,_=cv2.aruco.detectMarkers(g,n.dictionary,parameters=n.params)
    nc=0; vis=img.copy()
    if mi is not None and len(mi)>0:
        cv2.aruco.drawDetectedMarkers(vis,mc,mi)
        _,cc,ci=cv2.aruco.interpolateCornersCharuco(mc,mi,g,n.board)
        nc=0 if ci is None else len(ci)
        if nc>0: cv2.aruco.drawDetectedCornersCharuco(vis,cc,ci,(0,255,0))
        if nc>=6:
            ok,rv,tv=cv2.aruco.estimatePoseCharucoBoard(cc,ci,n.board,n.K,n.dist,None,None)
            if ok: cv2.drawFrameAxes(vis,n.K,n.dist,rv,tv,0.05)
    cv2.putText(vis,f"corners:{nc} markers:{0 if mi is None else len(mi)}",(15,35),
                cv2.FONT_HERSHEY_SIMPLEX,0.9,(0,255,0) if nc>=6 else (0,0,255),2)
    p=os.path.join(EV,f"{name}_c{nc}.jpg"); cv2.imwrite(p,vis)
    print(f"RESULT corners={nc} markers={0 if mi is None else len(mi)} saved={p}")
    rclpy.try_shutdown()

if __name__=="__main__": main()
