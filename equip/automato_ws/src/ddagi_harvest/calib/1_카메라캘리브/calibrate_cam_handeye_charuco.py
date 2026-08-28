#!/usr/bin/env python3
"""
차루코 1장으로 A(카메라 내부) + B(핸드아이 카메라↔base) 동시 캘리브
====================================================================
판(ChArUco)을 테이블에 딱 고정 → 팔이 여러 자세로 자동 순회하며 촬영.
카메라가 그리퍼에 달려있어(eye-in-hand) 판이 저절로 다양한 각도로 찍힘 →
그 한 세트로 아래 둘을 한 번에 뽑는다:

  A) 카메라 내부값 K, dist  =  cv2.calibrateCamera        ← "픽셀 ↔ 3D 광선"
  B) 카메라↔그리퍼 변환 X   =  cv2.calibrateHandEye(AX=XB) ← "카메라좌표 ↔ 로봇좌표"

핵심 순서(수학): A 먼저(K 없이는 판의 3D 자세를 못 구함) → 그 K로 각 자세 판자세
solvePnP → 그 짝들을 handeye 로. 물리 작업은 "판 고정 + 로봇 자세순회" 딱 한 번.

⭐ 차루코를 쓰는 이유: 판을 화면 구석·가장자리까지 밀어도(내부 캘리브 왜곡값이
   가장자리에서 결정됨) 일부 잘려도 인식됨. 일반 체커판은 조금만 잘려도 프레임 통째 버림.

────────────────────────────────────────────────────────────────────
사용법
  # 0) 판 PNG 생성 → A4 100%로 인쇄 → 딱딱한 판에 평평하게 부착 → square 실측!
  python3 calibrate_cam_handeye_charuco.py --make-board board_charuco.png

  # 1) 캘리브 (ROS2, 팔 자동순회). 인쇄 후 잰 실측 square/marker(mm) 반드시 전달
  source /opt/ros/jazzy/setup.bash && export ROS_DOMAIN_ID=20
  python3 calibrate_cam_handeye_charuco.py --square 30.0 --marker 22.0

  결과:  cam_calib.npz        (A: K, dist)  ← auto_scan_pick 등에서 불러다 씀
         handeye_charuco.json (B: R,t,규약)  ← cam_point_to_base 용
────────────────────────────────────────────────────────────────────
⚠ 팔이 자동으로 움직입니다. 주변 비우고, 판은 절대 움직이지 말 것.
"""
import argparse
import json
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from std_msgs.msg import String

# ── 차루코 판 규격 (한 판만 만들어 계속 씀) ──────────────────────────
SQUARES_X, SQUARES_Y = 7, 5           # 판의 사각형 개수(가로, 세로)
DICT_ID = cv2.aruco.DICT_4X4_50       # 마커 사전
IMG_W, IMG_H = 1280, 720              # D435 컬러 해상도 (A·B 내내 고정)

# 캘리브용 기준 관찰자세(기존 검증 성공 자세) — 판(책상)이 화면에 꽉 참
PVIEW = [6.24, 15.73, -77.6, -18.92, 2.9, 3.6]
MOVE_WAIT = 6.0
FRESH_SEC = 2.0
EULER_CANDIDATES = ["xyz", "zyx", "XYZ", "ZYX", "ZYZ"]   # pymycobot rx,ry,rz 규약 후보
HE_METHODS = {"TSAI": cv2.CALIB_HAND_EYE_TSAI, "PARK": cv2.CALIB_HAND_EYE_PARK,
              "HORAUD": cv2.CALIB_HAND_EYE_HORAUD, "DANIILIDIS": cv2.CALIB_HAND_EYE_DANIILIDIS}


def collection_poses():
    """PVIEW 주변 30자세: J1 좌우 ±42° × (거리/높이/틸트) — 회전 다양성 + 판을 화면
    구석까지 밀어 내부 캘리브 왜곡값까지 잘 잡히게."""
    deltas = [(0, 0, 0, 0), (6, -8, 10, 0), (-6, 8, -12, 0),
              (0, -6, 8, 14), (0, 6, -8, -14), (8, 0, -10, 8)]
    poses = []
    for dj1 in (-42, -21, 0, 21, 42):
        for (d2, d3, d4, d5) in deltas:
            p = list(PVIEW)
            p[0] += dj1; p[1] += d2; p[2] += d3; p[3] += d4; p[4] += d5
            poses.append([round(v, 2) for v in p])
    return poses


# ── ChArUco 래퍼 (OpenCV 4.7+ 신 API, 4.6 이하 자동 폴백) ─────────────
class Charuco:
    def __init__(self, square_mm, marker_mm):
        aruco = cv2.aruco
        self.dictionary = aruco.getPredefinedDictionary(DICT_ID)
        # 단위를 mm로 만들어 → 판자세 tvec 이 mm 로 나옴(로봇 좌표 mm 와 일치)
        try:
            self.board = aruco.CharucoBoard((SQUARES_X, SQUARES_Y), square_mm, marker_mm,
                                            self.dictionary)
            self.detector = aruco.CharucoDetector(self.board)
            self.new_api = True
        except AttributeError:                                   # OpenCV ≤ 4.6
            self.board = aruco.CharucoBoard_create(SQUARES_X, SQUARES_Y, square_mm,
                                                   marker_mm, self.dictionary)
            self.new_api = False

    def detect(self, gray):
        """→ (charuco_corners, charuco_ids) 또는 (None, None)"""
        aruco = cv2.aruco
        if self.new_api:
            cc, ci, _, _ = self.detector.detectBoard(gray)
        else:
            mc, mi, _ = aruco.detectMarkers(gray, self.dictionary)
            if mi is None or len(mi) == 0:
                return None, None
            _, cc, ci = aruco.interpolateCornersCharuco(mc, mi, gray, self.board)
        if ci is None or len(ci) < 6:        # 코너 6개 미만이면 자세 신뢰 X
            return None, None
        return cc, ci

    def obj_img_points(self, cc, ci):
        """charuco 코너 → (objP[mm], imgP[px]) 짝. calibrateCamera/solvePnP 공용."""
        if self.new_api:
            return self.board.matchImagePoints(cc, ci)          # (objP, imgP)
        # 구 API: chessboardCorners 테이블에서 id로 objP 조회
        obj = self.board.chessboardCorners[ci.flatten()].reshape(-1, 1, 3)
        return obj.astype(np.float32), cc.astype(np.float32)

    def draw(self, img, cc, ci):
        out = img.copy()
        cv2.aruco.drawDetectedCornersCharuco(out, cc, ci)
        return out


def make_board_png(path, square_mm, marker_mm):
    ch = Charuco(square_mm, marker_mm)
    # A4 300DPI ≈ 2480x3508 중 판 영역만. mm→px = *300/25.4
    px = lambda mm: int(round(mm * 300 / 25.4))
    w, h = px(square_mm * SQUARES_X), px(square_mm * SQUARES_Y)
    margin = px(square_mm) // 2
    if hasattr(ch.board, "generateImage"):                     # OpenCV 4.7+
        img = ch.board.generateImage((w, h), marginSize=margin)
    else:                                                       # OpenCV ≤ 4.6
        img = ch.board.draw((w, h), marginSize=margin, borderBits=1)
    cv2.imwrite(path, img)
    print("판 저장: %s  (%dx%d px, A4 100%%로 인쇄 → square 실측 후 --square 로 전달)"
          % (path, w, h))


# ── 변환 유틸 ────────────────────────────────────────────────────────
def T_from_Rt(R, t):
    T = np.eye(4); T[:3, :3] = R; T[:3, 3] = np.asarray(t).ravel()
    return T


def grip_T(coords, seq):
    x, y, z, rx, ry, rz = coords
    R = Rotation.from_euler(seq, [rx, ry, rz], degrees=True).as_matrix()
    return T_from_Rt(R, [x, y, z])


# ── ROS2 팔 링크 (기존과 동일) ───────────────────────────────────────
class ArmLink(Node):
    def __init__(self):
        super().__init__('cam_handeye_charuco')
        self.pub = self.create_publisher(String, '/automato/manual_cmd', 10)
        self.create_subscription(String, '/automato/arm_coords', self._on_coords, 10)
        self.coords, self.coords_time = None, 0.0

    def _on_coords(self, msg):
        try:
            c = json.loads(msg.data).get("coords")
            if c and len(c) == 6:
                self.coords, self.coords_time = c, time.time()
        except Exception:
            pass

    def move_angles(self, joints):
        m = String(); m.data = "angles:" + json.dumps(joints); self.pub.publish(m)

    def cmd(self, c):
        m = String(); m.data = c; self.pub.publish(m)

    def fresh_coords(self, wait=3.0):
        t0 = time.time()
        while time.time() - t0 < wait:
            if self.coords and time.time() - self.coords_time < FRESH_SEC:
                return list(self.coords)
            time.sleep(0.1)
        return None


class ColorCam:
    def __init__(self):
        import pyrealsense2 as rs
        self.pipe = rs.pipeline(); cfg = rs.config()
        cfg.enable_stream(rs.stream.color, IMG_W, IMG_H, rs.format.bgr8, 30)
        self.pipe.start(cfg)

    def grab(self, tries=8):
        img = None
        for _ in range(tries):
            f = self.pipe.wait_for_frames().get_color_frame()
            if f:
                img = np.asanyarray(f.get_data())
        return img

    def stop(self):
        self.pipe.stop()


# ── A: 내부 캘리브 ───────────────────────────────────────────────────
def calibrate_intrinsic(ch, all_cc, all_ci):
    """모아둔 charuco 코너들 → K, dist (여러 자세=판이 화면 곳곳에 찍힘)"""
    objps, imgps = [], []
    for cc, ci in zip(all_cc, all_ci):
        op, ip = ch.obj_img_points(cc, ci)
        if len(op) >= 6:
            objps.append(op.astype(np.float32)); imgps.append(ip.astype(np.float32))
    if len(objps) < 8:
        print("⚠ 내부 캘리브용 자세 < 8 — 판이 잘 보이는 자세 더 필요"); return None, None
    rms, K, dist, _, _ = cv2.calibrateCamera(objps, imgps, (IMG_W, IMG_H), None, None)
    print("\n[A] 내부 캘리브 완료  RMS 재투영오차 = %.3f px  (자세 %d개)" % (rms, len(objps)))
    print("    fx=%.1f fy=%.1f  cx=%.1f cy=%.1f" % (K[0, 0], K[1, 1], K[0, 2], K[1, 2]))
    np.savez("cam_calib.npz", K=K, dist=dist, rms=rms, image_size=(IMG_W, IMG_H))
    print("    저장: cam_calib.npz")
    return K, dist


# ── B: 핸드아이 (규약×방법 전수 → 판 고정성으로 자가검증) ─────────────
def solve_handeye(samples, K, dist, out_path, square_mm):
    print("\n[B] 핸드아이 풀이: 규약 %d × 방법 %d — 판 고정성(퍼짐)으로 최적 선택"
          % (len(EULER_CANDIDATES), len(HE_METHODS)))
    best = None
    for seq in EULER_CANDIDATES:
        Rg, tg, Rb, tb = [], [], [], []
        for s in samples:
            T = grip_T(s["coords"], seq)
            Rg.append(T[:3, :3]); tg.append(T[:3, 3])
            Rb.append(np.array(s["R_board"])); tb.append(np.array(s["t_board"]))
        for mname, mflag in HE_METHODS.items():
            try:
                Rx, tx = cv2.calibrateHandEye(Rg, tg, Rb, tb, method=mflag)
            except cv2.error:
                continue
            X = T_from_Rt(Rx, tx.ravel())
            pts = [ (grip_T(s["coords"], seq) @ X @
                     T_from_Rt(np.array(s["R_board"]), s["t_board"]))[:3, 3] for s in samples ]
            spread = float(np.linalg.norm(np.std(np.array(pts), axis=0)))
            print("  %-4s %-10s 판위치 퍼짐 = %7.1f mm" % (seq, mname, spread))
            if best is None or spread < best["spread"]:
                best = {"seq": seq, "method": mname, "spread": spread,
                        "R": Rx.tolist(), "t": tx.ravel().tolist()}
    if not best:
        print("풀이 실패"); return
    print("\n★ 최적: 규약=%s 방법=%s 퍼짐=%.1fmm%s"
          % (best["seq"], best["method"], best["spread"],
             "  ⚠ 큼 — 판 평탄도/실측 square/자세 다양성 확인" if best["spread"] > 30 else ""))
    json.dump({"R_cam2grip": best["R"], "t_cam2grip": best["t"], "euler_seq": best["seq"],
               "method": best["method"], "spread_mm": best["spread"], "square_mm": square_mm,
               "n_samples": len(samples), "ts": time.time(),
               "K": K.tolist(), "dist": dist.ravel().tolist()},
              open(out_path, "w"), indent=2)
    print("    저장: %s" % out_path)


def cam_point_to_base(cam_xyz_mm, coords, he):
    """런타임: cam점(mm) → base점(mm)"""
    X = T_from_Rt(np.array(he["R_cam2grip"]), he["t_cam2grip"])
    T = grip_T(coords, he["euler_seq"]) @ X
    return (T @ np.append(np.asarray(cam_xyz_mm, float), 1.0))[:3]


def run(args, arm):
    import os, datetime
    ev = "evidence/%s_camB_charuco" % datetime.datetime.now().strftime("%Y-%m-%d_%H%M")
    os.makedirs(ev, exist_ok=True)
    ch = Charuco(args.square, args.marker)
    cam = ColorCam()
    poses = collection_poses()
    all_cc, all_ci, samples = [], [], []
    print("자세 %d개 순회 — ⚠ 팔 자동 이동! 판 고정! 증거→ %s/" % (len(poses), ev))
    try:
        for i, p in enumerate(poses, 1):
            print("[%2d/%d] 이동 %s" % (i, len(poses), p))
            arm.move_angles(p); time.sleep(MOVE_WAIT)
            img = cam.grab()
            if img is None:
                print("      프레임 실패"); continue
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            cc, ci = ch.detect(gray)
            if cc is None:
                cv2.imwrite("%s/pose%02d_fail.jpg" % (ev, i), img)   # 실패장면도 기록
                print("      판 안 보임 → 건너뜀"); continue
            c = arm.fresh_coords()
            if c is None:
                print("      팔 좌표 미수신 → 건너뜀"); continue
            cv2.imwrite("%s/pose%02d_ok.jpg" % (ev, i), ch.draw(img, cc, ci))
            all_cc.append(cc); all_ci.append(ci)
            samples.append({"joints": p, "coords": c, "cc": cc.tolist(), "ci": ci.tolist()})
            print("      ✓ 코너 %d개 수집 (%d번째 자세)" % (len(ci), len(samples)))
    finally:
        cam.stop()
    print("\n수집 완료: %d/%d 자세 성공" % (len(samples), len(poses)))
    if len(samples) < 8:
        print("⚠ 8개 미만 — 판을 카메라가 더 잘 보는 곳으로 옮겨 재실행"); return

    # ── A 먼저 (내부 K) ──
    K, dist = calibrate_intrinsic(ch, all_cc, all_ci)
    if K is None:
        return
    # ── 그 K로 각 자세 판자세(cam←board) → B ──
    for s in samples:
        op, ip = ch.obj_img_points(np.array(s["cc"], np.float32), np.array(s["ci"]))
        ok, rvec, tvec = cv2.solvePnP(op, ip, K, dist)
        R, _ = cv2.Rodrigues(rvec)
        s["R_board"], s["t_board"] = R.tolist(), tvec.ravel().tolist()
    solve_handeye(samples, K, dist, args.out, args.square)
    arm.cmd("observe")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--make-board", metavar="PNG", help="차루코 판 PNG 생성 후 종료")
    ap.add_argument("--square", type=float, default=30.0, help="사각형 한 변(mm), 인쇄 후 실측!")
    ap.add_argument("--marker", type=float, default=22.0, help="마커 한 변(mm), 인쇄 후 실측!")
    ap.add_argument("--out", default="handeye_charuco.json")
    args = ap.parse_args()

    if args.make_board:
        make_board_png(args.make_board, args.square, args.marker); return

    rclpy.init()
    arm = ArmLink()
    threading.Thread(target=lambda: rclpy.spin(arm), daemon=True).start()
    print("로봇 좌표 수신 대기…")
    if not arm.fresh_coords(wait=10):
        print("⚠ /automato/arm_coords 미수신 — dg_control_node/도메인 확인"); return
    print("✅ 로봇 연결 OK")
    input("⚠ 팔이 자동으로 움직입니다. 주변 비우고 Enter → ")
    try:
        run(args, arm)
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
