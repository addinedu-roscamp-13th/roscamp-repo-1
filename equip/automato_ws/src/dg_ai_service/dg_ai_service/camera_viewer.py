#!/usr/bin/env python3
"""D435 RGB/Depth 스트림 뷰어.

CameraStream 으로 받은 컬러/깊이 프레임을 좌우로 나란히 붙여
하나의 창에 보여준다. 깊이는 pyrealsense2 colorizer 로 컬러맵을
입혀 눈으로 확인하기 쉽게 표시한다.

실행:
  PYTHONPATH=src/dg_ai_service .venv/bin/python -m dg_ai_service.camera_viewer

종료: 뷰어 창에서 q 또는 ESC.
"""
import argparse

import cv2
import numpy as np
import pyrealsense2 as rs

from dg_ai_service.camera_stream import CameraStream

WINDOW_NAME = 'D435 RGB | Depth'


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description='D435 RGB/Depth 좌우 분할 뷰어')
    parser.add_argument('--width', type=int, default=640)
    parser.add_argument('--height', type=int, default=480)
    parser.add_argument('--fps', type=int, default=30)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    cam = CameraStream(width=args.width, height=args.height, fps=args.fps)
    colorizer = rs.colorizer()

    print(f'[camera_viewer] D435 스트림 시작: {args.width}x{args.height}@{args.fps}fps '
          f'(종료: 뷰어 창에서 q 또는 ESC)')
    try:
        while True:
            color_frame, depth_frame = cam.get_frames()
            if not color_frame or not depth_frame:
                continue

            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(colorizer.colorize(depth_frame).get_data())

            combined = np.hstack((color_image, depth_image))
            cv2.imshow(WINDOW_NAME, combined)

            key = cv2.waitKey(1) & 0xFF
            if key in (ord('q'), 27):  # 27 == ESC
                break
            if cv2.getWindowProperty(WINDOW_NAME, cv2.WND_PROP_VISIBLE) < 1:
                break
    except KeyboardInterrupt:
        pass
    finally:
        cam.stop()
        cv2.destroyAllWindows()
        print('[camera_viewer] 종료')


if __name__ == '__main__':
    main()
