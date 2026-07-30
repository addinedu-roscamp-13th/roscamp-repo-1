#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
② 카메라↔베이스 핸드아이 캘리브 (eye-in-hand, WiFi로 로봇 제어)
================================================================
카메라가 그리퍼에 달림. 보드를 테이블에 '고정'하고, 로봇이 여러 자세로 움직이며
그 보드를 본다. 각 자세마다:
  - 카메라가 본 보드 자세  = cv2.solvePnP        (로컬 카메라, 공장 내부값)
  - 그때 로봇 자세         = /automato/arm_coords (paramiko로 읽음)
를 짝지어 모아 cv2.calibrateHandEye 로 '그리퍼↔카메라' 변환 X 를 구한다.

pymycobot의 rx,ry,rz 오일러 규약이 애매 → 여러 규약을 다 풀고, "보드는 고정"
이라는 사실로 자가검증(계산한 보드위치 퍼짐 최소인 규약 채택).

로봇: aac0 @raspi.local, 토픽 /automato/{manual_cmd,arm_coords}, ROS_DOMAIN_ID=20
보드: DICT_5X5_100, 5x7, square 30mm, marker 23mm  (네 보드)
"""
import json, os, time, math
import numpy as np, cv2
import pyrealsense2 as rs
import paramiko

ROBOT_IP="raspi.local"; USER="jetcobot"; PW="1"
SQUARES=(5,7); DICT=cv2.aruco.DICT_5X5_100; SQ=0.030; MK=0.023
SAVE=os.path.expanduser("~/Desktop/tomato_pkg_extract/deploy/evidence/2026-07-16_②핸드아이")
os.makedirs(SAVE, exist_ok=True)

# ---- 로봇 통신 (paramiko) ----
class Robot:
    def __init__(s):
        s.c=paramiko.SSHClient(); s.c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        s.c.connect(ROBOT_IP,username=USER,password=PW,timeout=10)
        s.pre="source /opt/ros/jazzy/setup.bash; export ROS_DOMAIN_ID=20; "
    def coords(s):
        o=s.c.exec_command(s.pre+"timeout 4 ros2 topic echo --once /automato/arm_coords 2>/dev/null")[1].read().decode()
        i=o.find("[")
        return json.loads("["+o[i+1:o.find("]")]+"]") if i>=0 else None
    def move_coords(s, xyzrpy, speed=40):
        cmd=(s.pre+f"ros2 topic pub -w 1 -t 3 /automato/manual_cmd std_msgs/msg/String "
             f"\"{{data: 'coords:{xyzrpy}'}}\" 2>/dev/null")
        s.c.exec_command(cmd)[1].read()
    def status(s):
        o=s.c.exec_command(s.pre+"timeout 3 ros2 topic echo --once /automato/arm_status 2>/dev/null")[1].read().decode()
        return o

# ---- 오일러(도) → 회전행렬 (여러 규약 후보) ----
def euler_R(rx,ry,rz,seq):
    rx,ry,rz=map(math.radians,(rx,ry,rz))
    def Rx(a):return np.array([[1,0,0],[0,math.cos(a),-math.sin(a)],[0,math.sin(a),math.cos(a)]])
    def Ry(a):return np.array([[math.cos(a),0,math.sin(a)],[0,1,0],[-math.sin(a),0,math.cos(a)]])
    def Rz(a):return np.array([[math.cos(a),-math.sin(a),0],[math.sin(a),math.cos(a),0],[0,0,1]])
    m={'x':Rx(rx),'y':Ry(ry),'z':Rz(rz)}
    R=np.eye(3)
    for ax in seq: R=R@m[ax]
    return R

def main():
    # 카메라 (컬러, 공장 내부값)
    pipe=rs.pipeline(); cfg=rs.config()
    cfg.enable_stream(rs.stream.color,1280,720,rs.format.bgr8,30)
    prof=pipe.start(cfg)
    intr=prof.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
    K=np.array([[intr.fx,0,intr.ppx],[0,intr.fy,intr.ppy],[0,0,1]]); dist=np.zeros(5)
    d=cv2.aruco.getPredefinedDictionary(DICT)
    board=cv2.aruco.CharucoBoard(SQUARES,SQ,MK,d); det=cv2.aruco.CharucoDetector(board)
    rob=Robot(); print("[i] 로봇 연결됨.  SPACE=캡처(로봇자세+보드자세)  C=계산  Q=종료")

    R_g2b,t_g2b,R_b2c,t_b2c=[],[],[],[]
    n=0
    try:
        while True:
            img=np.asanyarray(pipe.wait_for_frames().get_color_frame().get_data())
            g=cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
            cc,ci,_,_=det.detectBoard(g)
            vis=img.copy(); nc=0 if ci is None else len(ci)
            pose_ok=False; rvec=tvec=None
            if nc>=6:
                cv2.aruco.drawDetectedCornersCharuco(vis,cc,ci)
                op,ip=board.matchImagePoints(cc,ci)
                if op is not None and len(op)>=6:
                    ok,rvec,tvec=cv2.solvePnP(op,ip,K,dist)
                    if ok: pose_ok=True; cv2.drawFrameAxes(vis,K,dist,rvec,tvec,0.05)
            cv2.putText(vis,f"corners:{nc} pose:{'OK' if pose_ok else 'X'}  captured:{n} (SPACE/C/Q)",
                        (15,35),cv2.FONT_HERSHEY_SIMPLEX,0.8,(0,255,0) if pose_ok else (0,0,255),2)
            cv2.imshow("handeye calib (SPACE/C/Q)",vis)
            k=cv2.waitKey(1)&0xFF
            if k==ord('q'): break
            elif k==ord(' '):
                if not pose_ok: print("  보드 자세 안 잡힘 — 보드가 잘 보이게"); continue
                co=rob.coords()
                if co is None: print("  로봇 자세 못 읽음"); continue
                Rg=euler_R(co[3],co[4],co[5],'xyz')       # 규약은 계산때 여러개 시도
                R_g2b.append((co[3],co[4],co[5]))          # 원각도 저장(규약 나중에)
                t_g2b.append(np.array(co[:3]))
                R_b2c.append(cv2.Rodrigues(rvec)[0]); t_b2c.append(tvec.ravel()*1000)
                n+=1
                cv2.imwrite(f"{SAVE}/pose{n:02d}.jpg",vis)
                print(f"  캡처 {n}: 로봇 {np.round(co,1).tolist()}  (이미지 저장)")
            elif k==ord('c'):
                if n<3: print(f"  {n}개 — 최소 3, 권장 8+"); continue
                solve(R_g2b,t_g2b,R_b2c,t_b2c)
    finally:
        pipe.stop(); cv2.destroyAllWindows(); rob.c.close()

def solve(euler_list,t_g2b,R_b2c,t_b2c):
    SEQS=["xyz","zyx","XYZ","ZYX","ZYZ"]  # 규약 후보
    METHODS={"TSAI":cv2.CALIB_HAND_EYE_TSAI,"PARK":cv2.CALIB_HAND_EYE_PARK,
             "HORAUD":cv2.CALIB_HAND_EYE_HORAUD,"DANIILIDIS":cv2.CALIB_HAND_EYE_DANIILIDIS}
    best=None
    for seq in SEQS:
        Rg=[euler_R(e[0],e[1],e[2], seq.lower() if seq.islower() else seq.lower()) for e in euler_list]
        tg=[t.reshape(3,1) for t in t_g2b]
        Rb=[R for R in R_b2c]; tb=[t.reshape(3,1) for t in t_b2c]
        for mn,mf in METHODS.items():
            try: R_x,t_x=cv2.calibrateHandEye(Rg,tg,Rb,tb,method=mf)
            except cv2.error: continue
            # 자가검증: 보드가 base에서 고정이면 각 자세 board위치 퍼짐 최소
            pts=[]
            for Rgi,tgi,Rbi,tbi in zip(Rg,tg,Rb,tb):
                Tg=np.eye(4); Tg[:3,:3]=Rgi; Tg[:3,3]=tgi.ravel()
                X=np.eye(4); X[:3,:3]=R_x; X[:3,3]=t_x.ravel()
                Tb=np.eye(4); Tb[:3,:3]=Rbi; Tb[:3,3]=tbi.ravel()
                pts.append((Tg@X@Tb)[:3,3])
            spread=float(np.std(np.array(pts),axis=0).mean())
            if best is None or spread<best["spread"]:
                best={"seq":seq,"method":mn,"R":R_x.tolist(),"t":t_x.ravel().tolist(),"spread_mm":spread}
    print(f"\n===== ② 핸드아이 결과 =====")
    print(f"  최적 규약={best['seq']} 방법={best['method']}  퍼짐(품질)={best['spread_mm']:.1f}mm")
    print(f"  t(카메라위치, mm)={np.round(best['t'],1).tolist()}")
    json.dump(best,open(f"{SAVE}/handeye_result.json","w"),ensure_ascii=False,indent=2)
    print(f"  저장: {SAVE}/handeye_result.json")

if __name__=="__main__":
    main()
