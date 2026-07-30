#!/usr/bin/env python3
"""TF 오일러 규약 진단 (노트북, 화면 필요) — get_coords의 rx,ry,rz 순서를 실측으로 찾기.

문제: camera→base 변환의 base←joint6 를 get_coords + 오일러로 만드는데, pymycobot의
오일러 규약(ZYX?)이 애매해 결과가 크게 어긋난다. 건수님은 ROS TF(FK)로 우회했다.
여기선 '정답'으로 역추적한다:
  ① 관측자세에서 토마토를 클릭 → 그 픽셀의 camera 좌표
  ② 그리퍼로 그 토마토를 직접 만져 get_coords → 진짜 base 위치(P_true)
  ③ 여러 오일러 순서로 camera_to_base 계산 → P_true에 가장 가까운 순서가 정답

  python3 tf_calibrate.py

마우스 좌클릭: 토마토 픽셀 선택(camera 좌표 저장)
키: m  그리퍼로 만진 위치 측정(서보풀기→드래그→키→get_coords)   s 스윕결과   q 종료
"""
import os
import sys

import numpy as np
import pyrealsense2 as rs

W, H, FPS = 640, 480, 30
ORDERS = ["XYZ", "XZY", "YXZ", "YZX", "ZXY", "ZYX"]
_click = {"uv": None}


def _on_mouse(event, x, y, flags, param):
    import cv2
    if event == cv2.EVENT_LBUTTONDOWN:
        _click["uv"] = (x if x < W else x - W, y)


def _sweep(cam, p_true, tf):
    """여러 오일러 순서로 camera→base 계산, P_true와의 거리 정렬 출력."""
    print("\n=== 오일러 순서 스윕 (P_true 에 가까울수록 정답) ===")
    print(f"  P_true(만진 위치) = [{p_true[0]:.1f}, {p_true[1]:.1f}, {p_true[2]:.1f}]")
    rows = []
    for o in ORDERS:
        b = tf.camera_to_base(cam, tf.OBSERVE_COORDS, euler_order=o)
        dist = float(np.linalg.norm(np.array(b) - np.array(p_true)))
        rows.append((dist, o, b))
    rows.sort()
    for dist, o, b in rows:
        mark = "  ← 최적" if (dist, o, b) == rows[0] else ""
        print(f"  {o}: base=[{b[0]:7.1f},{b[1]:7.1f},{b[2]:7.1f}]  오차 {dist:6.1f}mm{mark}")
    print(f"\n→ 추정 정답 순서: {rows[0][1]} (오차 {rows[0][0]:.1f}mm)")
    print("  (오차엔 그리퍼 손끝-플랜지 TCP~109mm가 섞여 있으니, 순서 판별용으로만)")


def main():
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    from ddagi_harvest.arm_backend import NetworkArm
    from ddagi_harvest import tf_transform as tf
    from ddagi_harvest import pick as pk
    try:
        import cv2
    except Exception:
        print("opencv 필요")
        return 1
    if len(rs.context().query_devices()) == 0:
        print("RealSense 없음")
        return 1

    ip = os.environ.get("ARM_IP", "192.168.3.12")
    print(f"팔 연결 → {ip}:9010")
    arm = NetworkArm(ip)
    print("관측자세 이동...")
    arm.move_angles(pk.OBSERVE_ANGLES, 30)

    pipe = rs.pipeline()
    cfg = rs.config()
    cfg.enable_stream(rs.stream.depth, W, H, rs.format.z16, FPS)
    cfg.enable_stream(rs.stream.color, W, H, rs.format.bgr8, FPS)
    pipe.start(cfg)
    align = rs.align(rs.stream.color)
    cv2.namedWindow("tf calibrate")
    cv2.setMouseCallback("tf calibrate", _on_mouse)

    cam = None
    uv = None
    print(__doc__)
    try:
        while True:
            frames = align.process(pipe.wait_for_frames())
            depth, color = frames.get_depth_frame(), frames.get_color_frame()
            if not depth or not color:
                continue
            intr = color.profile.as_video_stream_profile().get_intrinsics()

            if _click["uv"] is not None:
                u, v = _click["uv"]; _click["uv"] = None
                d = depth.get_distance(u, v)
                if d == 0:
                    print(f"({u},{v}) depth 없음")
                else:
                    cam = [c * 1000 for c in rs.rs2_deproject_pixel_to_point(intr, [u, v], d)]
                    uv = (u, v)
                    print(f"\n토마토 선택 ({u},{v}) depth={d*100:.1f}cm  "
                          f"camera=[{cam[0]:.1f},{cam[1]:.1f},{cam[2]:.1f}]")
                    print("  → 이제 그리퍼로 이 토마토를 만지고 'm' 을 누르세요")

            img = np.asanyarray(color.get_data()).copy()
            dimg = cv2.applyColorMap(
                cv2.convertScaleAbs(np.asanyarray(depth.get_data()), alpha=0.03),
                cv2.COLORMAP_JET)
            if uv:
                cv2.drawMarker(img, uv, (0, 0, 255), cv2.MARKER_TILTED_CROSS, 22, 2)
            cv2.putText(img, "click tomato | m:measure  s:sweep  q:quit",
                        (10, H - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            cv2.imshow("tf calibrate", np.hstack([img, dimg]))

            k = cv2.waitKey(1) & 0xFF
            if k == ord("q"):
                break
            elif k == ord("m"):
                if cam is None:
                    print("먼저 토마토를 클릭하세요"); continue
                print("\n서보 풉니다 — 그리퍼 끝을 그 토마토에 대고(받치기!) 창에서 아무 키.")
                arm.release_servos()
                cv2.waitKey(0)
                p_true = arm.get_coords()[:3]
                arm.focus_servos()
                print(f"측정 P_true = [{p_true[0]:.1f}, {p_true[1]:.1f}, {p_true[2]:.1f}]")
                globals()["_LAST"] = (cam, p_true)
                _sweep(cam, p_true, tf)
            elif k == ord("s"):
                if "_LAST" in globals():
                    _sweep(*globals()["_LAST"], tf)
                else:
                    print("먼저 클릭 후 'm' 측정")
    finally:
        arm.close(); pipe.stop(); cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
