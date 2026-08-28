#!/usr/bin/env python3
"""
풀 손-눈 캘리브레이션 (방법 B) — eye-in-hand, ROS2, 자동 수집
==============================================================
"그리퍼↔카메라" 고정 변환 X 를 구한다. X 만 있으면 팔이 어떤 자세에 있든
  base점 = T_base←grip(현재 팔좌표) · X · cam점
으로 즉시 변환 → 임의 자세 스캔/서보잉의 기반.

원리 (AX=XB):
  체커판을 테이블에 고정해 두고, 팔을 여러 자세로 움직이며
  ① 로봇좌표(base←gripper)  ② 카메라가 본 체커판 자세(cam←board) 를 짝으로 수집.
  자세 i→j 로 움직일 때 그리퍼의 변화 A 와 카메라가 본 변화 B 는
  A·X = X·B 를 만족 → OpenCV cv2.calibrateHandEye 로 X 를 푼다.

pymycobot 의 rx,ry,rz 오일러 규약이 문서마다 달라 → 5개 후보 규약을 전부 풀고
"체커판은 고정" 이라는 사실로 자가검증(모든 자세에서 계산한 board 위치의 퍼짐이
최소인 규약 채택). 퍼짐(mm)이 곧 캘리브 품질 지표.

수집(자동): 팔이 18개 자세를 저속으로 순회하며 촬영 — 손작업 없음!
  ⚠ 팔이 계속 움직입니다. 주변 비우고, 체커판은 절대 움직이지 말 것.

실행(노트북):
  source /opt/ros/jazzy/setup.bash
  export ROS_DOMAIN_ID=20
  python3 calibrate_handeye_full.py                 # 수집+풀기 (YOLO 불필요)
  python3 calibrate_handeye_full.py --square 19.8   # 인쇄 후 실측값으로
  PYTHONPATH=~/sam3/.venv/lib/python3.12/site-packages \
      python3 calibrate_handeye_full.py --verify    # 토마토로 검증(YOLO 필요)
"""
import argparse
import json
import threading
import time

import cv2
import numpy as np
from d435_capture import natural_wb
import pyrealsense2 as rs
import rclpy
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from std_msgs.msg import String

BOARD_COLS, BOARD_ROWS = 8, 5          # 내부 코너 수 (checkerboard_8x5_18mm)
# 캘리브용 기준 관찰자세: 원래 PVIEW에서 J4를 35° 숙여 체커판(책상)이 화면에 꽉 참
# (2026-07-03 스냅샷 튜닝으로 확정 — board_check4.jpg 검출 성공 자세)
PVIEW = [6.24, 15.73, -77.6, -18.92, 2.9, 3.6]
MOVE_WAIT = 6.0                        # 자세 이동+정착 대기(초)
FRESH_SEC = 2.0
EULER_CANDIDATES = ["xyz", "zyx", "XYZ", "ZYX", "ZYZ"]   # scipy 표기(소문자=extrinsic)
HE_METHODS = {"TSAI": cv2.CALIB_HAND_EYE_TSAI, "PARK": cv2.CALIB_HAND_EYE_PARK,
              "HORAUD": cv2.CALIB_HAND_EYE_HORAUD, "DANIILIDIS": cv2.CALIB_HAND_EYE_DANIILIDIS}


def collection_poses():
    """PVIEW 주변 18개 자세: J1(좌우) x (거리/높이/틸트) 조합 — 회전 다양성 확보."""
    deltas = [(0, 0, 0, 0), (6, -8, 10, 0), (-6, 8, -12, 0),
              (0, -6, 8, 14), (0, 6, -8, -14), (8, 0, -10, 8)]
    poses = []
    # 좌우 ±42°까지 확장(30자세) — 작업영역 가장자리 외삽 오차 축소 (2026-07-03 사고 반영)
    for dj1 in (-42, -21, 0, 21, 42):
        for (d2, d3, d4, d5) in deltas:
            p = list(PVIEW)
            p[0] += dj1; p[1] += d2; p[2] += d3; p[3] += d4; p[4] += d5
            poses.append([round(v, 2) for v in p])
    return poses


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--square", type=float, default=18.0, help="체커판 사각형 한 변(mm), 인쇄 후 실측!")
    p.add_argument("--out", default="handeye_full.json")
    p.add_argument("--verify", action="store_true", help="저장된 X로 토마토 다각도 검증")
    p.add_argument("--model", default="tomato_4cls.pt")
    return p.parse_args()


class ArmLink(Node):
    def __init__(self):
        super().__init__('handeye_full_calibrator')
        self.pub = self.create_publisher(String, '/automato/manual_cmd', 10)
        self.create_subscription(String, '/automato/arm_coords', self._on_coords, 10)
        self.coords, self.coords_time = None, 0.0

    def _on_coords(self, msg):
        try:
            d = json.loads(msg.data)
            c = d.get("coords")
            if c and len(c) == 6:
                self.coords, self.coords_time = c, time.time()
        except Exception:
            pass

    def cmd(self, c):
        m = String(); m.data = c
        self.pub.publish(m)

    def move_angles(self, joints):
        self.cmd("angles:" + json.dumps(joints))

    def fresh_coords(self, wait=3.0):
        t0 = time.time()
        while time.time() - t0 < wait:
            if self.coords and time.time() - self.coords_time < FRESH_SEC:
                return list(self.coords)
            time.sleep(0.1)
        return None


# ---------------- 변환 유틸 ----------------
def T_from_Rt(R, t):
    T = np.eye(4); T[:3, :3] = R; T[:3, 3] = np.asarray(t).ravel()
    return T


def grip_T(coords, seq):
    """pymycobot get_coords → T_base←gripper (규약 seq 가정)"""
    x, y, z, rx, ry, rz = coords
    R = Rotation.from_euler(seq, [rx, ry, rz], degrees=True).as_matrix()
    return T_from_Rt(R, [x, y, z])


class BoardCam:
    """D435 컬러 + 체커판 PnP (YOLO 불필요)"""

    def __init__(self, square_mm):
        self.pipe = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
        prof = self.pipe.start(cfg)
        intr = prof.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        self.K = np.array([[intr.fx, 0, intr.ppx], [0, intr.fy, intr.ppy], [0, 0, 1]])
        self.dist = np.array(intr.coeffs[:5], float)
        n = BOARD_COLS * BOARD_ROWS
        self.obj = np.zeros((n, 3), np.float32)
        self.obj[:, :2] = np.mgrid[0:BOARD_COLS, 0:BOARD_ROWS].T.reshape(-1, 2) * square_mm

    def board_pose(self, tries=8):
        """체커판의 cam←board (R,t[mm]) — 몇 프레임 시도, 실패 시 None.
        self.last_raw(원본)·self.last_annot(코너표시) 에 마지막 프레임 보관(증거 저장용)."""
        self.last_raw, self.last_annot = None, None
        for _ in range(tries):
            frames = self.pipe.wait_for_frames()
            img = np.asanyarray(frames.get_color_frame().get_data())
            self.last_raw = img.copy()
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            ok, corners = cv2.findChessboardCorners(
                gray, (BOARD_COLS, BOARD_ROWS),
                cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE)
            if not ok:
                continue
            corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1),
                                       (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.01))
            ok, rvec, tvec = cv2.solvePnP(self.obj, corners, self.K, self.dist)
            if ok:
                annot = img.copy()
                cv2.drawChessboardCorners(annot, (BOARD_COLS, BOARD_ROWS), corners, True)
                self.last_annot = annot
                R, _ = cv2.Rodrigues(rvec)
                return R, tvec.ravel()
        return None

    def stop(self):
        self.pipe.stop()


def solve_all(samples, out_path, square_mm):
    """규약 x 방법 전수 풀기 → 체커판 고정성(퍼짐)으로 최적 선택"""
    print("\n=== 풀이: 오일러 규약 %d개 × 방법 %d개 ===" % (len(EULER_CANDIDATES), len(HE_METHODS)))
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
            # 자가검증: 모든 자세에서 base←board 계산 → 고정이어야 함
            pts = []
            for s in samples:
                Tb = grip_T(s["coords"], seq) @ X @ T_from_Rt(np.array(s["R_board"]), s["t_board"])
                pts.append(Tb[:3, 3])
            spread = float(np.linalg.norm(np.std(np.array(pts), axis=0)))
            print("  %-4s %-10s 체커판위치 퍼짐 = %7.1f mm" % (seq, mname, spread))
            if best is None or spread < best["spread"]:
                best = {"seq": seq, "method": mname, "spread": spread,
                        "R": Rx.tolist(), "t": tx.ravel().tolist()}
    if not best:
        print("풀이 실패"); return
    print("\n★ 최적: 규약=%s 방법=%s 퍼짐=%.1fmm" % (best["seq"], best["method"], best["spread"]))
    if best["spread"] > 30:
        print("  ⚠ 퍼짐이 큼 — 체커판 평탄도/실측 square/자세 다양성 확인 후 재수집 권장")
    data = {"R_cam2grip": best["R"], "t_cam2grip": best["t"],
            "euler_seq": best["seq"], "method": best["method"],
            "spread_mm": best["spread"], "square_mm": square_mm,
            "n_samples": len(samples), "ts": time.time()}
    with open(out_path, "w") as f:
        json.dump(data, f, indent=2)
    print("저장: %s (자세 %d개 사용)" % (out_path, len(samples)))


def cam_point_to_base(cam_xyz_mm, coords, he):
    """런타임 변환: cam점(mm) → base점(mm). he = handeye_full.json dict"""
    X = T_from_Rt(np.array(he["R_cam2grip"]), he["t_cam2grip"])
    T = grip_T(coords, he["euler_seq"]) @ X
    p = np.append(np.asarray(cam_xyz_mm, float), 1.0)
    return (T @ p)[:3]


def run_collect(args, arm):
    import os, datetime
    ev = "evidence/%s_collect" % datetime.datetime.now().strftime("%Y-%m-%d_%H%M")
    os.makedirs(ev, exist_ok=True)
    cam = BoardCam(args.square)
    poses = collection_poses()
    samples = []
    print("자세 %d개 순회 시작 — ⚠ 팔이 자동으로 움직입니다! 체커판 고정!" % len(poses))
    print("증거 이미지 자동 저장: %s/" % ev)
    try:
        for i, p in enumerate(poses, 1):
            print("[%2d/%d] 이동 %s" % (i, len(poses), p))
            arm.move_angles(p)
            time.sleep(MOVE_WAIT)
            bp = cam.board_pose()
            if bp is None:
                if cam.last_raw is not None:      # 실패 장면도 기록(원인 추적용)
                    cv2.imwrite("%s/pose%02d_fail.jpg" % (ev, i), cam.last_raw)
                print("      체커판 안 보임 → 건너뜀"); continue
            c = arm.fresh_coords()
            if c is None:
                print("      팔 좌표 미수신 → 건너뜀"); continue
            R_b, t_b = bp
            cv2.imwrite("%s/pose%02d_ok_annot.jpg" % (ev, i), cam.last_annot)   # 코너표시본
            cv2.imwrite("%s/pose%02d_ok_color.jpg" % (ev, i), cam.last_raw)     # 원본컬러
            cv2.imwrite("%s/pose%02d_ok_natural.jpg" % (ev, i), natural_wb(cam.last_raw))  # 자연색
            samples.append({"joints": p, "coords": c,
                            "R_board": R_b.tolist(), "t_board": t_b.tolist()})
            print("      ✓ 수집 (%d개째)  board_z=%.0fmm" % (len(samples), t_b[2]))
    finally:
        cam.stop()
    print("\n수집 완료: %d/%d 자세 성공" % (len(samples), len(poses)))
    with open("handeye_full_samples.json", "w") as f:
        json.dump(samples, f)
    if len(samples) < 8:
        print("⚠ 8개 미만 — 체커판 위치를 카메라가 더 잘 보이는 곳으로 옮기고 재실행하세요.")
        return
    solve_all(samples, args.out, args.square)
    arm.cmd("observe")


def run_verify(args, arm):
    """토마토 1개를 두고 서로 다른 3자세에서 검출 → base 좌표 일치도 확인"""
    import os, datetime
    from td435_common import TomatoD435
    ev = "evidence/%s_verify" % datetime.datetime.now().strftime("%Y-%m-%d_%H%M")
    os.makedirs(ev, exist_ok=True)
    print("증거 이미지 자동 저장: %s/" % ev)
    he = json.load(open(args.out))
    cam = TomatoD435(args.model, conf=0.4)
    test_poses = [list(PVIEW), None, None]
    p2 = list(PVIEW); p2[0] -= 20; p2[3] += 10; test_poses[1] = p2
    p3 = list(PVIEW); p3[0] += 20; p3[4] += 10; test_poses[2] = p3
    results = []
    try:
        for i, p in enumerate(test_poses, 1):
            print("[검증 %d/3] 이동 %s" % (i, [round(v, 1) for v in p]))
            arm.move_angles(p); time.sleep(MOVE_WAIT)
            samples = []; last_img = None
            for _ in range(10):
                img, dets = cam.read()
                b = cam.best(dets)
                if b:
                    samples.append(b["cam_xyz"]); last_img = img
            if last_img is not None:
                cv2.imwrite("%s/verify_pose%d.jpg" % (ev, i), last_img)  # 검출박스 포함
                cv2.imwrite("%s/verify_pose%d_natural.jpg" % (ev, i), natural_wb(last_img))
            if not samples:
                print("      토마토 미검출"); continue
            cam_mm = np.median(np.array(samples), axis=0) * 1000.0
            c = arm.fresh_coords()
            if not c:
                print("      팔 좌표 미수신"); continue
            base = cam_point_to_base(cam_mm, c, he)
            results.append(base)
            print("      base 좌표 = [%.1f, %.1f, %.1f] mm" % tuple(base))
    finally:
        cam.stop()
    if len(results) >= 2:
        arr = np.array(results)
        spread = np.linalg.norm(arr.max(0) - arr.min(0))
        print("\n★ 자세 간 좌표 차이(최대): %.1f mm — %s"
              % (spread, "✅ 훌륭 (<15mm)" if spread < 15 else
                 ("양호 (<30mm) — 서보잉으로 커버 가능" if spread < 30 else "⚠ 큼 — 재캘리브 권장")))
    arm.cmd("observe")


def main():
    args = parse_args()
    rclpy.init()
    arm = ArmLink()
    threading.Thread(target=lambda: rclpy.spin(arm), daemon=True).start()
    print("로봇 좌표 수신 대기…")
    if not arm.fresh_coords(wait=10):
        print("⚠ /automato/arm_coords 미수신 — dg_control_node/도메인 확인"); return
    print("✅ 로봇 연결 OK")
    input("⚠ 팔이 자동으로 움직입니다. 주변을 비우고 Enter → ")
    try:
        if args.verify:
            run_verify(args, arm)
        else:
            run_collect(args, arm)
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
