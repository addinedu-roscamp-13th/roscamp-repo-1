#!/usr/bin/env python3
"""RealSense 실물 프로브 — 한 픽셀을 camera(optical) 3D 좌표로.

TF 검증·개발용. 정렬된 컬러+깊이에서 지정 픽셀(기본 중앙)을 역투영해
camera_color_optical_frame 좌표(mm)를 출력하고, 표시한 컬러 프레임을 저장한다.
(실제 토마토 검출/YOLO는 AI 서비스=손민호 담당. 이건 좌표를 확인하는 테스트 도구.)

  python3 camera_probe.py            # 중앙 픽셀
  python3 camera_probe.py 320 240    # 지정 픽셀 (u v)

주의: AI가 Ddagi로 주는 좌표도 이 optical frame이어야 tf_transform과 맞물린다.
"""
import sys

import numpy as np
import pyrealsense2 as rs

W, H, FPS = 640, 480, 30


def main():
    u = int(sys.argv[1]) if len(sys.argv) > 1 else W // 2
    v = int(sys.argv[2]) if len(sys.argv) > 2 else H // 2

    if len(rs.context().query_devices()) == 0:
        print("RealSense 장치 없음 — USB 연결 확인")
        return 1

    pipe = rs.pipeline()
    cfg = rs.config()
    cfg.enable_stream(rs.stream.depth, W, H, rs.format.z16, FPS)
    cfg.enable_stream(rs.stream.color, W, H, rs.format.bgr8, FPS)
    profile = pipe.start(cfg)
    try:
        align = rs.align(rs.stream.color)
        for _ in range(15):          # 자동노출 안정화
            pipe.wait_for_frames()
        frames = align.process(pipe.wait_for_frames())
        depth, color = frames.get_depth_frame(), frames.get_color_frame()
        if not depth or not color:
            print("프레임 획득 실패")
            return 1

        intr = color.profile.as_video_stream_profile().get_intrinsics()
        d = depth.get_distance(u, v)          # m
        if d == 0:                             # 유효 깊이 없으면 주변 중앙값
            arr = np.asanyarray(depth.get_data())
            patch = arr[max(0, v - 3):v + 4, max(0, u - 3):u + 4]
            nz = patch[patch > 0]
            d = float(np.median(nz)) * depth.get_units() if nz.size else 0.0

        pt = rs.rs2_deproject_pixel_to_point(intr, [u, v], d)  # m, optical frame
        pt_mm = [c * 1000.0 for c in pt]

        print(f"intrinsics: fx={intr.fx:.1f} fy={intr.fy:.1f} "
              f"ppx={intr.ppx:.1f} ppy={intr.ppy:.1f}  {W}x{H}")
        print(f"pixel=({u},{v})  depth={d*100:.1f}cm")
        print(f"camera 좌표(optical, mm) = "
              f"[{pt_mm[0]:.1f}, {pt_mm[1]:.1f}, {pt_mm[2]:.1f}]")
        if d == 0:
            print("주의: depth=0 (유효 깊이 없음 — 반사/범위밖/너무 가까움)")

        # 표시용 프레임 저장(픽셀 십자 표시)
        try:
            import cv2
            img = np.asanyarray(color.get_data()).copy()
            cv2.drawMarker(img, (u, v), (0, 0, 255), cv2.MARKER_CROSS, 20, 2)
            cv2.imwrite("camera_probe.png", img)
            print("프레임 저장: camera_probe.png")
        except Exception:
            pass
    finally:
        pipe.stop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
