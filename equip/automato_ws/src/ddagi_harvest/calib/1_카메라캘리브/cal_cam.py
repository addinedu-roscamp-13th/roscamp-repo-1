# 카메라가 본 '가장 확신 높은 토마토'의 카메라좌표(mm)를 여러 프레임 중앙값으로 출력
import numpy as np, cv2, pyrealsense2 as rs, onnxruntime as ort
from detect_onnx import detect
W,H=1280,720
sess=ort.InferenceSession("cherry_tomato.onnx",providers=["CPUExecutionProvider"])
pipe=rs.pipeline();cfg=rs.config()
cfg.enable_stream(rs.stream.color,W,H,rs.format.bgr8,15)
cfg.enable_stream(rs.stream.depth,W,H,rs.format.z16,15)
prof=pipe.start(cfg);align=rs.align(rs.stream.color)
intr=prof.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
def med(depth,cx,cy,k=2):
    v=[depth.get_distance(cx+a,cy+b) for a in range(-k,k+1) for b in range(-k,k+1) if depth.get_distance(cx+a,cy+b)>0]
    return float(np.median(v)) if v else 0.0
for _ in range(20): pipe.wait_for_frames()
S=[]
for _ in range(15):
    f=align.process(pipe.wait_for_frames())
    color,depth=f.get_color_frame(),f.get_depth_frame()
    if not color or not depth: continue
    dets=detect(sess,np.asanyarray(color.get_data()),0.5)
    if not dets: continue
    (x1,y1,x2,y2),sc=max(dets,key=lambda d:d[1])
    cx,cy=int((x1+x2)/2),int((y1+y2)/2); z=med(depth,cx,cy)
    if z<=0: continue
    X,Y,Z=rs.rs2_deproject_pixel_to_point(intr,[cx,cy],z)
    S.append([X*1000,Y*1000,Z*1000])
pipe.stop()
if not S: print("CAM_NONE")
else:
    m=np.median(S,axis=0); print(f"CAMXYZ {m[0]:.1f} {m[1]:.1f} {m[2]:.1f}")
