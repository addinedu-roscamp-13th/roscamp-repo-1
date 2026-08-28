#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
auto_tcp — 잡힌 방울토마토로 TCP 자동 측정 (③TCP 캘리브)
========================================================
그리퍼에 물린 토마토를 여러 자세(J6 고정, J1/J2/J4/J5만 변경)에서 카메라로
검출→ TCP_joint6 = X·(camera→tomato). J6 고정이라 모든 자세에서 같아야 함(검증).
끝에 J6 회전분을 빼서 J6-무관 TCP_flange 도 출력.
로봇: manual_cmd angles 로만 이동(안전). 그리퍼 잡은 상태 유지.
실행: ROS_DOMAIN_ID=20 python3 auto_tcp.py
"""
import time, yaml, os
import numpy as np, cv2, pyrealsense2 as rs
import rclpy
from std_msgs.msg import String

CAL="/home/ane/.ros2/easy_handeye2/calibrations/jetcobot_handeye.calib"
EV=os.path.expanduser("~/Desktop/tomato_pkg_extract/deploy/evidence/2026-07-20_③TCP캘리브")
J6=30.76  # 고정(그리퍼가 J6 뒤라 J6는 고정해야 TCP 일정)
POSES=[[0.35,-42,-1.3,-1.6,11,J6],   # 현재(기준)
       [9,   -42,-1.3,-1.6,11,J6],   # J1+
       [-8,  -42,-1.3,-1.6,11,J6],   # J1-
       [0.35,-42, 8, -1.6,11,J6],    # J3+ (추가)
       [0.35,-42,-10,-1.6,11,J6],    # J3- (추가)
       [0.35,-38,-1.3,-1.6,11,J6],   # J2+
       [0.35,-42,-1.3, 8, 11,J6],    # J4+
       [0.35,-42,-1.3,-1.6,20,J6]]   # J5+

def quat2R(x,y,z,w):
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])

def detect_closest_red(bgr, df):
    hsv=cv2.cvtColor(bgr,cv2.COLOR_BGR2HSV)
    m1=cv2.inRange(hsv,(0,90,60),(12,255,255)); m2=cv2.inRange(hsv,(168,90,60),(180,255,255))
    mask=cv2.morphologyEx(m1|m2,cv2.MORPH_OPEN,np.ones((5,5),np.uint8))
    mask=cv2.morphologyEx(mask,cv2.MORPH_CLOSE,np.ones((7,7),np.uint8))
    cnts,_=cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    best=None
    for c in cnts:
        a=cv2.contourArea(c)
        if a<300: continue
        (x,y),r=cv2.minEnclosingCircle(c); x,y,r=int(x),int(y),int(r)
        if r<7: continue
        peri=cv2.arcLength(c,True); circ=4*3.14159*a/(peri*peri) if peri>0 else 0
        if circ<0.45: continue
        ds=[df.get_distance(min(639,max(0,x+dx)),min(479,max(0,y+dy)))
            for dx in range(-3,4) for dy in range(-3,4)]
        ds=[d for d in ds if 0.08<d<1.2]
        if not ds: continue
        depth=float(np.median(ds))
        if best is None or depth<best[3]: best=(x,y,r,depth)
    return best

def main():
    os.makedirs(EV,exist_ok=True)
    cal=yaml.safe_load(open(CAL)); tx=cal['transform']['translation']; rx=cal['transform']['rotation']
    X=np.eye(4); X[:3,:3]=quat2R(rx['x'],rx['y'],rx['z'],rx['w']); X[:3,3]=[tx['x'],tx['y'],tx['z']]
    rclpy.init(); node=rclpy.create_node('auto_tcp'); pub=node.create_publisher(String,'/automato/manual_cmd',10)
    def cmd(s):
        m=String(); m.data=s
        for _ in range(3): pub.publish(m); rclpy.spin_once(node,timeout_sec=0.05); time.sleep(0.05)
    pipe=rs.pipeline(); cfg=rs.config()
    cfg.enable_stream(rs.stream.color,640,480,rs.format.bgr8,30)
    cfg.enable_stream(rs.stream.depth,640,480,rs.format.z16,30)
    prof=pipe.start(cfg); align=rs.align(rs.stream.color)
    intr=prof.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
    print("=== 그리퍼 닫기(토마토 잡기 확실히) ===",flush=True)
    cmd("grip_close"); time.sleep(2.0)
    tcps=[]
    for i,p in enumerate(POSES,1):
        cmd("angles:"+str(p)); time.sleep(5.5)
        det=None
        for _ in range(20):
            fr=align.process(pipe.wait_for_frames())
            bgr=np.asanyarray(fr.get_color_frame().get_data()); df=fr.get_depth_frame()
            det=detect_closest_red(bgr,df)
            if det: break
            time.sleep(0.05)
        if not det:
            print(f"  자세{i} {p}: 토마토 검출실패",flush=True); continue
        u,v,r,depth=det
        cam=rs.rs2_deproject_pixel_to_point(intr,[u,v],depth)
        tcp=(X@np.array([cam[0],cam[1],cam[2],1.0]))[:3]*1000
        tcps.append(tcp)
        vis=bgr.copy(); cv2.circle(vis,(u,v),r,(0,255,0),2); cv2.circle(vis,(u,v),3,(0,0,255),-1)
        cv2.putText(vis,f"pose{i} depth={depth*100:.1f}cm TCP=[{tcp[0]:.0f},{tcp[1]:.0f},{tcp[2]:.0f}]mm",
                    (10,28),cv2.FONT_HERSHEY_SIMPLEX,0.6,(0,255,255),2)
        cv2.imwrite(EV+f"/자동측정_{i:02d}.jpg",vis)
        print(f"  자세{i} {p}: depth={depth*100:.1f}cm  TCP_j6=[{tcp[0]:.1f},{tcp[1]:.1f},{tcp[2]:.1f}]mm",flush=True)
    pipe.stop()
    if len(tcps)>=3:
        arr=np.array(tcps); mean=arr.mean(0); spread=float(np.linalg.norm(arr.std(0)))
        # J6 회전분 제거 → flange(J6-무관) 기준
        th=np.radians(J6)
        def Rx(a):c,s=np.cos(a),np.sin(a);return np.array([[1,0,0],[0,c,-s],[0,s,c]])
        def Rz(a):c,s=np.cos(a),np.sin(a);return np.array([[c,-s,0],[s,c,0],[0,0,1]])
        T=np.eye(4); T[:3,:3]=Rx(-1.5708)@Rz(th); T[:3,3]=[0,0.0456,0]
        tcp_f=(np.linalg.inv(T)@np.array([mean[0]/1000,mean[1]/1000,mean[2]/1000,1.0]))[:3]*1000
        print("\n===== TCP 측정 결과 =====",flush=True)
        print(f"  TCP_joint6 (J6={J6}° 기준) = [{mean[0]:.1f}, {mean[1]:.1f}, {mean[2]:.1f}] mm",flush=True)
        print(f"  자세간 퍼짐(작을수록 정확): {spread:.1f} mm  ({len(tcps)}자세)",flush=True)
        print(f"  판정: {'✅ 우수(<5mm)' if spread<5 else '✅ 양호(<10mm)' if spread<10 else '△ 보통' if spread<20 else '❌ 재측정'}",flush=True)
        print(f"  TCP_flange (J6무관) = [{tcp_f[0]:.1f}, {tcp_f[1]:.1f}, {tcp_f[2]:.1f}] mm",flush=True)
        open(EV+"/TCP결과.txt","w").write(
            f"TCP_joint6_mm(J6={J6}) = [{mean[0]:.1f}, {mean[1]:.1f}, {mean[2]:.1f}]\n"
            f"TCP_flange_mm(J6무관) = [{tcp_f[0]:.1f}, {tcp_f[1]:.1f}, {tcp_f[2]:.1f}]\n"
            f"spread={spread:.2f}mm  poses={len(tcps)}\n")
    else:
        print("측정 실패(자세 부족)",flush=True)
    rclpy.try_shutdown()

if __name__=="__main__": main()
