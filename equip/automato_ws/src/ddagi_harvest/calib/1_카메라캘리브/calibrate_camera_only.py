#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
카메라 내부 캘리브레이션 (로봇 없이 — 보드를 손으로 들고 여러 각도로 촬영)
======================================================================
D435 컬러 카메라의 내부값(fx, fy, cx, cy, 왜곡)을 ChArUco 보드로 직접 측정한다.
인텔 공장값(intel_factory_calib.json)과 비교 가능.

원리:
  ChArUco 보드를 여러 각도/위치로 카메라에 비추면, 각 프레임에서 보드 코너의
  (3D 실제좌표[mm] ↔ 2D 픽셀좌표[px]) 짝을 얻는다. 이 짝을 여러 장 모아
  cv2.calibrateCamera 에 넣으면 카메라 내부값 K, 왜곡 dist 가 나온다.

사용:
  python3 calibrate_camera_only.py --square 30 --marker 22
  창에서:  SPACE=현재화면 저장   C=캘리브 계산(12장↑)   Q=종료
  ※ 보드를 상/하/좌/우/기울여 골고루 12~20장 모을 것 (구석·기울임 다양할수록 정확)
"""
import argparse, json, os
import cv2
import numpy as np
import pyrealsense2 as rs

# 보드 규격 — deploy/calibrate_cam_handeye_charuco.py 와 동일(DICT_4X4_50, 7x5)
SQUARES_X, SQUARES_Y = 7, 5
DICT_ID = cv2.aruco.DICT_4X4_50
W, H = 1280, 720   # 인텔 공장값과 같은 해상도로 비교

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--square", type=float, default=30.0, help="사각형 한 변(mm), 인쇄 후 실측")
    ap.add_argument("--marker", type=float, default=22.0, help="마커 한 변(mm)")
    a = ap.parse_args()

    dictionary = cv2.aruco.getPredefinedDictionary(DICT_ID)
    board = cv2.aruco.CharucoBoard((SQUARES_X, SQUARES_Y),
                                   a.square/1000.0, a.marker/1000.0, dictionary)
    detector = cv2.aruco.CharucoDetector(board)

    pipe = rs.pipeline(); cfg = rs.config()
    cfg.enable_stream(rs.stream.color, W, H, rs.format.bgr8, 30)
    pipe.start(cfg)
    print("[i] SPACE=저장  C=캘리브  Q=종료.  보드를 여러 각도로 비추세요.")

    all_corners, all_ids = [], []   # 모은 화면들의 charuco 코너/id
    try:
        while True:
            fr = pipe.wait_for_frames().get_color_frame()
            img = np.asanyarray(fr.get_data())
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            ch_corners, ch_ids, _, _ = detector.detectBoard(gray)

            vis = img.copy()
            n = 0 if ch_ids is None else len(ch_ids)
            if n > 0:
                cv2.aruco.drawDetectedCornersCharuco(vis, ch_corners, ch_ids)
            # 안내 문구
            cv2.putText(vis, f"corners:{n}  captured:{len(all_corners)}  (SPACE/C/Q)",
                        (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.9,
                        (0, 255, 0) if n >= 6 else (0, 0, 255), 2)
            cv2.imshow("camera calibration (SPACE/C/Q)", vis)
            k = cv2.waitKey(1) & 0xFF

            if k == ord('q'):
                break
            elif k == ord(' '):                       # 현재 화면 저장
                if n >= 6:
                    all_corners.append(ch_corners); all_ids.append(ch_ids)
                    print(f"  저장 {len(all_corners)}장 (코너 {n})")
                else:
                    print("  코너 부족(<6) — 보드가 더 잘 보이게")
            elif k == ord('c'):                       # 캘리브 계산
                if len(all_corners) < 12:
                    print(f"  {len(all_corners)}장 — 12장 이상 모아주세요"); continue
                objps, imgps = [], []
                for cc, ci in zip(all_corners, all_ids):
                    op, ip = board.matchImagePoints(cc, ci)     # 3D[mm]↔2D[px] 짝
                    if op is not None and len(op) >= 6:
                        objps.append(op); imgps.append(ip)
                rms, K, dist, _, _ = cv2.calibrateCamera(objps, imgps, (W, H), None, None)
                print("\n===== 내 카메라 캘리브 결과 =====")
                print(f"  재투영오차 RMS = {rms:.3f} px  (자세 {len(objps)}장)")
                print(f"  fx={K[0,0]:.2f} fy={K[1,1]:.2f}  cx={K[0,2]:.2f} cy={K[1,2]:.2f}")
                print(f"  왜곡 dist = {dist.ravel().round(4).tolist()}")
                # 인텔 공장값과 비교
                fp = os.path.join(os.path.dirname(__file__), "intel_factory_calib.json")
                if os.path.exists(fp):
                    F = json.load(open(fp))["① 카메라 내부값 (RGB 컬러)"]
                    print("\n  ── 인텔 공장값과 비교 ──")
                    print(f"    fx: 내값 {K[0,0]:.1f}  vs 인텔 {F['fx']}")
                    print(f"    fy: 내값 {K[1,1]:.1f}  vs 인텔 {F['fy']}")
                    print(f"    cx: 내값 {K[0,2]:.1f}  vs 인텔 {F['cx']}")
                    print(f"    cy: 내값 {K[1,2]:.1f}  vs 인텔 {F['cy']}")
                out = {"설명": "내가 ChArUco 보드로 직접 측정한 카메라 내부값",
                       "해상도": f"{W}x{H}", "재투영오차_px": round(float(rms), 3),
                       "자세수": len(objps),
                       "fx": round(float(K[0,0]),2), "fy": round(float(K[1,1]),2),
                       "cx": round(float(K[0,2]),2), "cy": round(float(K[1,2]),2),
                       "왜곡계수": dist.ravel().round(5).tolist()}
                op = os.path.join(os.path.dirname(__file__), "my_camera_calib.json")
                json.dump(out, open(op, "w"), ensure_ascii=False, indent=2)
                np.savez(os.path.join(os.path.dirname(__file__), "my_cam_calib.npz"),
                         K=K, dist=dist, rms=rms)
                print(f"\n  저장: my_camera_calib.json + my_cam_calib.npz")
    finally:
        pipe.stop(); cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
