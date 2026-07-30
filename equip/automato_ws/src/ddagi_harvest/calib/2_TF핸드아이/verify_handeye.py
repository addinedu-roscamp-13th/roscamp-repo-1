#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
verify_handeye — 핸드아이 캘리브 정확도 검증 (자가일관성, 카메라 SDK 직접)
==========================================================================
보드는 base에 고정 → 캘리브(X=joint6→camera)가 정확하면
  base→board = (base→joint6)·X·(camera→board)  가 모든 자세에서 같아야 한다.
카메라를 SDK로 직접 읽어 보드검출(camera→board), 로봇 TF(g_base→joint6)와 곱해
여러 자세의 base→board 퍼짐(std)을 잰다. 퍼짐 작을수록 정확.
실행: ROS_DOMAIN_ID=20 python3 verify_handeye.py  (charuco 안 떠있어도 됨)
"""
import time, yaml
import numpy as np, cv2, pyrealsense2 as rs
import paramiko
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

CAL = "/home/ane/.ros2/easy_handeye2/calibrations/jetcobot_handeye.calib"
POSES = [[0,-40,0,0,0,30],[8,-40,0,0,8,30],[-8,-40,0,0,8,30],
         [0,-40,0,0,12,30],[10,-42,0,0,6,30],[-10,-42,0,0,6,30],[6,-38,0,0,4,30]]
SQUARES=(5,7); DICT_ID=cv2.aruco.DICT_5X5_100; SQ,MK=0.030,0.023

def quat2R(x,y,z,w):
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
def T(t,q): M=np.eye(4); M[:3,:3]=quat2R(*q); M[:3,3]=t; return M
def tf2T(tr):
    t=[tr.transform.translation.x,tr.transform.translation.y,tr.transform.translation.z]
    q=[tr.transform.rotation.x,tr.transform.rotation.y,tr.transform.rotation.z,tr.transform.rotation.w]
    return T(t,q)

class V(Node):
    def __init__(self):
        super().__init__('verify')
        self.buf=Buffer(); TransformListener(self.buf,self)
        # 카메라 SDK
        self.pipe=rs.pipeline(); cfg=rs.config(); cfg.enable_stream(rs.stream.color,1280,720,rs.format.bgr8,30)
        prof=self.pipe.start(cfg); intr=prof.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        self.K=np.array([[intr.fx,0,intr.ppx],[0,intr.fy,intr.ppy],[0,0,1]]); self.dist=np.array(intr.coeffs)
        self.dic=cv2.aruco.getPredefinedDictionary(DICT_ID)
        self.board=cv2.aruco.CharucoBoard_create(SQUARES[0],SQUARES[1],SQ,MK,self.dic)
        try: self.pr=cv2.aruco.DetectorParameters_create()
        except: self.pr=cv2.aruco.DetectorParameters()
        self.ssh=paramiko.SSHClient(); self.ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        self.ssh.connect("192.168.3.12",username="jetcobot",password="1",timeout=15,banner_timeout=20)
        self.pre="source /opt/ros/jazzy/setup.bash; export ROS_DOMAIN_ID=20; "
    def move(self,a):
        self.ssh.exec_command(self.pre+f"ros2 topic pub --once /automato/manual_cmd std_msgs/msg/String \"data: 'angles:{a}'\" 2>/dev/null")[1].read()
    def spin(self,s):
        t0=time.time()
        while time.time()-t0<s: rclpy.spin_once(self,timeout_sec=0.05)
    def cam_board(self):
        for _ in range(8): f=self.pipe.wait_for_frames().get_color_frame()
        img=np.asanyarray(f.get_data()); g=cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        mc,mi,_=cv2.aruco.detectMarkers(g,self.dic,parameters=self.pr)
        if mi is None or len(mi)==0: return None,0
        _,cc,ci=cv2.aruco.interpolateCornersCharuco(mc,mi,g,self.board)
        nc=0 if ci is None else len(ci)
        if nc<6: return None,nc
        ok,rv,tv=cv2.aruco.estimatePoseCharucoBoard(cc,ci,self.board,self.K,self.dist,None,None)
        if not ok: return None,nc
        M=np.eye(4); M[:3,:3]=cv2.Rodrigues(rv)[0]; M[:3,3]=tv.ravel(); return M,nc

def main():
    cal=yaml.safe_load(open(CAL)); tx=cal['transform']['translation']; rx=cal['transform']['rotation']
    X=T([tx['x'],tx['y'],tx['z']],[rx['x'],rx['y'],rx['z'],rx['w']])
    rclpy.init(); n=V()
    print("=== 정확도 검증 (카메라 SDK 직접, 7자세) ===",flush=True)
    pts=[]
    for i,pose in enumerate(POSES,1):
        n.move(pose); n.spin(6.0)
        cb,nc=n.cam_board()
        if cb is None: print(f"  자세{i} {pose}: 보드 검출실패 (코너 {nc})",flush=True); n.spin(0.2); continue
        try: bj=tf2T(n.buf.lookup_transform('g_base','joint6',Time()))
        except Exception as e: print(f"  자세{i}: 로봇TF 실패 {e}",flush=True); continue
        bb=bj@X@cb; pts.append(bb[:3,3])
        print(f"  자세{i} {pose} (코너{nc}): base→board=[{bb[0,3]*1000:.1f}, {bb[1,3]*1000:.1f}, {bb[2,3]*1000:.1f}]mm",flush=True)
        n.spin(0.2)
    pts=np.array(pts)
    if len(pts)>=3:
        std=pts.std(0)*1000; sprd=float(np.linalg.norm(pts.std(0)))*1000; mean=pts.mean(0)*1000
        print("\n===== 검증 결과 =====",flush=True)
        print(f"  base→board 평균: [{mean[0]:.1f}, {mean[1]:.1f}, {mean[2]:.1f}] mm  (같아야 정확)",flush=True)
        print(f"  ★ 종합 오차(3D std): {sprd:.1f} mm  (자세{len(pts)}개)",flush=True)
        print(f"  판정: {'✅ 우수(<5mm)' if sprd<5 else '✅ 양호(<10mm)' if sprd<10 else '△ 보통(<20mm)' if sprd<20 else '❌ 재캘리브 권장'}",flush=True)
    else: print("검증 실패(자세부족)",flush=True)
    try: n.pipe.stop(); n.ssh.close()
    except: pass
    rclpy.try_shutdown()

if __name__=="__main__": main()
