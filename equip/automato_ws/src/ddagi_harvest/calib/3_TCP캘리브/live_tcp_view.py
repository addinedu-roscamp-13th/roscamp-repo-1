#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
live_tcp_view — 실시간 카메라 + 토마토검출 + TCP 실시간 측정 (③TCP 캘리브)
=========================================================================
그리퍼에 잡힌 방울토마토(카메라에 가장 가까운 빨강)를 검출→ joint6 기준
그리퍼끝 좌표(TCP) = X · (camera→tomato) 를 실시간 표시. 여러 자세에서 s로
저장하면 평균±표준편차로 수렴 확인. 로봇 안 움직임(측정만). 그리퍼만 o/c.
  키: o=그리퍼 열기  c=닫기  s=현재값 저장  r=리셋  q=종료
실행: ROS_DOMAIN_ID=20 python3 live_tcp_view.py
"""
import time, yaml, os
import numpy as np, cv2, pyrealsense2 as rs
import rclpy
from std_msgs.msg import String

CAL="/home/ane/.ros2/easy_handeye2/calibrations/jetcobot_handeye.calib"
EV=os.path.expanduser("~/Desktop/tomato_pkg_extract/deploy/evidence/2026-07-20_③TCP캘리브")

def quat2R(x,y,z,w):
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])

def detect_closest_red(bgr, df):
    """빨간 둥근 물체 중 카메라에 가장 가까운(깊이 최소) 것 = 잡힌 토마토"""
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
        if best is None or depth<best[3]:
            best=(x,y,r,depth)
    return best

def main():
    os.makedirs(EV,exist_ok=True)
    cal=yaml.safe_load(open(CAL)); tx=cal['transform']['translation']; rx=cal['transform']['rotation']
    X=np.eye(4); X[:3,:3]=quat2R(rx['x'],rx['y'],rx['z'],rx['w']); X[:3,3]=[tx['x'],tx['y'],tx['z']]
    rclpy.init(); node=rclpy.create_node('live_tcp'); pub=node.create_publisher(String,'/automato/manual_cmd',10)
    def cmd(s): m=String(); m.data=s; pub.publish(m); rclpy.spin_once(node,timeout_sec=0.05)
    pipe=rs.pipeline(); cfg=rs.config()
    cfg.enable_stream(rs.stream.color,640,480,rs.format.bgr8,30)
    cfg.enable_stream(rs.stream.depth,640,480,rs.format.z16,30)
    prof=pipe.start(cfg); align=rs.align(rs.stream.color)
    intr=prof.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
    samples=[]; last=None
    cv2.namedWindow("live TCP (o=open c=close s=save r=reset q=quit)",cv2.WINDOW_NORMAL)
    cv2.resizeWindow("live TCP (o=open c=close s=save r=reset q=quit)",900,700)
    print("=== 실시간 TCP 뷰어 시작 ===  o=열기 c=닫기 s=저장 r=리셋 q=종료",flush=True)
    while True:
        fr=align.process(pipe.wait_for_frames())
        bgr=np.asanyarray(fr.get_color_frame().get_data()); df=fr.get_depth_frame()
        vis=bgr.copy(); det=detect_closest_red(bgr,df); last=None
        if det:
            u,v,r,depth=det
            cam=rs.rs2_deproject_pixel_to_point(intr,[u,v],depth)
            tcp=(X@np.array([cam[0],cam[1],cam[2],1.0]))[:3]*1000
            last=tcp
            cv2.circle(vis,(u,v),r,(0,255,0),2); cv2.circle(vis,(u,v),3,(0,0,255),-1)
            cv2.putText(vis,f"depth={depth*100:.1f}cm",(u-50,v-r-8),cv2.FONT_HERSHEY_SIMPLEX,0.6,(0,255,0),2)
            cv2.putText(vis,f"TCP(joint6): [{tcp[0]:.0f}, {tcp[1]:.0f}, {tcp[2]:.0f}] mm",
                        (12,30),cv2.FONT_HERSHEY_SIMPLEX,0.7,(0,255,255),2)
            cv2.putText(vis,f"|offset|={np.linalg.norm(tcp):.0f} mm",
                        (12,58),cv2.FONT_HERSHEY_SIMPLEX,0.65,(0,255,255),2)
        else:
            cv2.putText(vis,"토마토(빨강) 검출안됨 - 그리퍼에 방울토마토 물리고 닫아줘",
                        (12,30),cv2.FONT_HERSHEY_SIMPLEX,0.55,(0,0,255),2)
        if samples:
            arr=np.array(samples); mean=arr.mean(0); std=float(np.linalg.norm(arr.std(0)))
            cv2.putText(vis,f"저장 {len(samples)}개  평균TCP=[{mean[0]:.0f},{mean[1]:.0f},{mean[2]:.0f}] 퍼짐={std:.1f}mm",
                        (12,86),cv2.FONT_HERSHEY_SIMPLEX,0.6,(255,200,0),2)
        cv2.imshow("live TCP (o=open c=close s=save r=reset q=quit)",vis)
        k=cv2.waitKey(1)&0xFF
        if k==ord('q'): break
        elif k==ord('o'): cmd("grip_open"); print("그리퍼 열기",flush=True)
        elif k==ord('c'): cmd("grip_close"); print("그리퍼 닫기",flush=True)
        elif k==ord('r'): samples=[]; print("리셋",flush=True)
        elif k==ord('s') and last is not None:
            samples.append(last.copy())
            cv2.imwrite(EV+f"/측정_{len(samples):02d}.jpg",vis)
            print(f"저장#{len(samples)} TCP=[{last[0]:.0f},{last[1]:.0f},{last[2]:.0f}]mm",flush=True)
    if samples:
        arr=np.array(samples); mean=arr.mean(0); std=float(np.linalg.norm(arr.std(0)))
        open(EV+"/TCP결과.txt","w").write(
            f"TCP_joint6_mm = [{mean[0]:.1f}, {mean[1]:.1f}, {mean[2]:.1f}]\n"
            f"samples={len(samples)}  spread_std={std:.2f}mm\n")
        print(f"\n★ 최종 TCP(joint6): [{mean[0]:.1f},{mean[1]:.1f},{mean[2]:.1f}]mm  퍼짐 {std:.1f}mm ({len(samples)}개)",flush=True)
    pipe.stop(); rclpy.try_shutdown()

if __name__=="__main__": main()
