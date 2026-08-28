#!/usr/bin/env python3
"""RealSense 라이브 뷰어 — 관측자세 프레이밍용 (노트북, 화면 필요).

카메라가 팔에 붙어 있어, 팔을 드래그하면 이 화면이 그 시야로 바뀐다.
토마토 베드가 잘 담기는 자세를 찾은 뒤, Pi에서 OBSERVE_ANGLES/get_coords를 캡처한다.

  python3 camera_view.py

화면: 컬러(중앙 십자 + 그 픽셀의 camera 좌표) | 깊이 컬러맵.
키:  s 프레임 저장(view_frame.png)   q 종료
디스플레이 없으면(SSH 등) 자동으로 1초마다 중앙 좌표를 텍스트로 출력.
"""
import time

import numpy as np
import pyrealsense2 as rs

W, H, FPS = 640, 480, 30


def _center_coord(depth, color, u, v):
    intr = color.profile.as_video_stream_profile().get_intrinsics()
    d = depth.get_distance(u, v)
    if d == 0:
        return None
    return rs.rs2_deproject_pixel_to_point(intr, [u, v], d)  # m, optical


def main():
    if len(rs.context().query_devices()) == 0:
        print("RealSense 장치 없음")
        return 1

    pipe = rs.pipeline()
    cfg = rs.config()
    cfg.enable_stream(rs.stream.depth, W, H, rs.format.z16, FPS)
    cfg.enable_stream(rs.stream.color, W, H, rs.format.bgr8, FPS)
    pipe.start(cfg)
    align = rs.align(rs.stream.color)
    u, v = W // 2, H // 2

    try:
        import cv2
        headless = False
    except Exception:
        cv2 = None
        headless = True

    last_print = 0.0
    try:
        while True:
            frames = align.process(pipe.wait_for_frames())
            depth, color = frames.get_depth_frame(), frames.get_color_frame()
            if not depth or not color:
                continue
            pt = _center_coord(depth, color, u, v)
            label = ("no-depth" if pt is None
                     else f"[{pt[0]*1000:.0f}, {pt[1]*1000:.0f}, {pt[2]*1000:.0f}]mm")

            if headless:
                if time.time() - last_print > 1.0:
                    print(f"center({u},{v}) camera(optical) = {label}")
                    last_print = time.time()
                continue

            img = np.asanyarray(color.get_data()).copy()
            dimg = cv2.applyColorMap(
                cv2.convertScaleAbs(np.asanyarray(depth.get_data()), alpha=0.03),
                cv2.COLORMAP_JET)
            cv2.drawMarker(img, (u, v), (0, 0, 255), cv2.MARKER_CROSS, 20, 2)
            cv2.putText(img, f"center {label}", (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            cv2.putText(img, "s: save   q: quit", (10, H - 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            cv2.imshow("observe framing (color | depth)",
                       np.hstack([img, dimg]))
            k = cv2.waitKey(1) & 0xFF
            if k == ord("q"):
                break
            if k == ord("s"):
                cv2.imwrite("view_frame.png", img)
                print("저장: view_frame.png")
    finally:
        pipe.stop()
        if not headless:
            import cv2
            cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
