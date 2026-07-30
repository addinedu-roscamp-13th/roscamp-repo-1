#!/usr/bin/env python3
"""관측자세 세팅 (노트북, 화면 필요) — 카메라 라이브 + 팔 원격조종을 한 화면에.

카메라가 팔에 붙어 있어 팔을 움직이면 이 화면이 그 시야로 바뀐다. 서보를 풀어
손으로 드래그하며 토마토 베드가 잘 담기는 자세를 찾고, 그 자리에서 관측자세를
캡처한다. 캡처는 OBSERVE_ANGLES(관절각)와 OBSERVE_COORDS(get_coords)를 함께 딴다
— 각각 pick.py 상수와 tf_transform의 base←joint6 에 들어간다.

  python3 observe_setup.py                        # 팔 IP 기본 192.168.3.12
  ARM_IP=192.168.x.x python3 observe_setup.py

키(카메라 창에 포커스):
  r  서보 풀기(드래그 가능 — 팔 받치기!)   f  서보 잠그기
  c  관측자세 캡처 (angles + coords)        s  현재 프레임 저장
  q  종료(서보 잠그고 캡처값 출력)

!! r 누르면 팔이 중력으로 처진다. 반드시 손으로 받친 뒤 드래그.
"""
import os

import numpy as np
import pyrealsense2 as rs

W, H, FPS = 640, 480, 30


def main():
    import sys
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    from ddagi_harvest.arm_backend import NetworkArm

    try:
        import cv2
    except Exception:
        print("opencv 필요 (화면 표시). pip install opencv-python")
        return 1
    if len(rs.context().query_devices()) == 0:
        print("RealSense 장치 없음")
        return 1

    ip = os.environ.get("ARM_IP", "192.168.3.12")
    print(f"팔 서버 연결 → {ip}:9010 ...")
    arm = NetworkArm(ip)
    print("연결 OK")

    pipe = rs.pipeline()
    cfg = rs.config()
    cfg.enable_stream(rs.stream.depth, W, H, rs.format.z16, FPS)
    cfg.enable_stream(rs.stream.color, W, H, rs.format.bgr8, FPS)
    pipe.start(cfg)
    align = rs.align(rs.stream.color)
    u, v = W // 2, H // 2

    servo_state = "locked"
    captured = None
    print(__doc__)

    try:
        while True:
            frames = align.process(pipe.wait_for_frames())
            depth, color = frames.get_depth_frame(), frames.get_color_frame()
            if not depth or not color:
                continue
            intr = color.profile.as_video_stream_profile().get_intrinsics()
            d = depth.get_distance(u, v)
            cc = (rs.rs2_deproject_pixel_to_point(intr, [u, v], d) if d else None)
            cc_txt = ("no-depth" if cc is None
                      else f"[{cc[0]*1000:.0f},{cc[1]*1000:.0f},{cc[2]*1000:.0f}]mm")

            img = np.asanyarray(color.get_data()).copy()
            dimg = cv2.applyColorMap(
                cv2.convertScaleAbs(np.asanyarray(depth.get_data()), alpha=0.03),
                cv2.COLORMAP_JET)
            cv2.drawMarker(img, (u, v), (0, 0, 255), cv2.MARKER_CROSS, 20, 2)
            color_state = (0, 200, 0) if servo_state == "locked" else (0, 165, 255)
            cv2.putText(img, f"center {cc_txt}", (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            cv2.putText(img, f"servo: {servo_state}", (10, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, color_state, 2)
            if captured:
                cv2.putText(img, "captured OK", (10, 75),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 0), 2)
            cv2.putText(img, "r:release f:lock c:capture s:save q:quit",
                        (10, H - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            cv2.imshow("observe setup (color | depth)", np.hstack([img, dimg]))

            k = cv2.waitKey(1) & 0xFF
            if k == ord("q"):
                break
            elif k == ord("r"):
                arm.release_servos()
                servo_state = "RELEASED (드래그—팔 받치기!)"
                print("서보 해제 — 손으로 받치고 드래그")
            elif k == ord("f"):
                arm.focus_servos()
                servo_state = "locked"
                print("서보 잠금")
            elif k == ord("c"):
                angles = arm.get_angles()
                coords = arm.get_coords()
                captured = {"angles": angles, "coords": coords}
                print(f"\n[캡처] OBSERVE_ANGLES = {angles}")
                print(f"       OBSERVE_COORDS = {coords}\n")
            elif k == ord("s"):
                cv2.imwrite("observe_frame.png", img)
                print("저장: observe_frame.png")
    finally:
        try:
            arm.focus_servos()
        except Exception:
            pass
        arm.close()
        pipe.stop()
        cv2.destroyAllWindows()

    if captured:
        print("=" * 56)
        print("# pick.py 에 붙여넣기:")
        print(f"OBSERVE_ANGLES = {[round(a,1) for a in captured['angles']]}")
        print("# tf_transform 관측자세 get_coords (내가 넣어줄게):")
        print(f"OBSERVE_COORDS = {[round(c,1) for c in captured['coords']]}")
        print("=" * 56)
    else:
        print("캡처 없이 종료됨")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
