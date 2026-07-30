#!/usr/bin/env python3
"""
손-눈 캘리브레이션 (eye-to-hand: 카메라 고정, 작업공간을 바라봄)
=================================================================
카메라가 보는 3D 위치(미터)를 로봇팔 좌표(mm)로 바꾸는 변환을 1회 측정.

원리: 같은 점을 (1)카메라로 인식한 좌표 와 (2)로봇팔 끝을 그 점에 직접 댔을 때의
      로봇 좌표 를 4군데 이상 짝지어 모으면, 둘 사이 강체변환을 풀 수 있다.

준비물: 방울토마토(또는 작고 빨간 물체) 1개. 카메라·로봇 모두 고정.

절차(각 측정점마다 반복, 최소 4점 / 권장 6점, 공간상 넓게 퍼뜨릴 것):
  1) 토마토를 카메라가 잘 보는 위치에 놓는다
  2) 터미널에서 Enter → 스크립트가 그 토마토의 카메라 3D좌표를 잡는다
  3) 로봇팔을 수동(손으로 잡고)으로 움직여 그리퍼 끝을 '같은 토마토 중심'에 댄다
  4) Enter → 그 순간 로봇 좌표를 읽어 짝으로 저장
  ... 4점 이상 모이면 's' 입력으로 저장

실행:
  python3 calibrate_handeye.py --model cherry_tomato.pt --out handeye.json
"""
import argparse
import json
import time

import numpy as np

from td435_common import TomatoD435, solve_rigid_transform


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--model", default="cherry_tomato.pt")
    p.add_argument("--out", default="handeye.json")
    p.add_argument("--port", default="/dev/ttyJETCOBOT")
    p.add_argument("--baud", type=int, default=1000000)
    p.add_argument("--conf", type=float, default=0.4)
    p.add_argument("--remote", default="", help="Pi 팔서버 'IP:포트' (소켓 방식). 비우면 로컬 시리얼")
    return p.parse_args()


def read_coords(mc, tries=5):
    """get_coords 가 가끔 None/빈값 → 재시도"""
    for _ in range(tries):
        c = mc.get_coords()
        if c and len(c) == 6:
            return c
        time.sleep(0.2)
    return None


def main():
    args = parse_args()
    if args.remote:
        from remote_arm import RemoteArm
        host, port = args.remote.split(":")
        mc = RemoteArm(host, int(port))
    else:
        from pymycobot.mycobot280 import MyCobot280
        mc = MyCobot280(args.port, args.baud)
        mc.thread_lock = True
    time.sleep(1)

    cam = TomatoD435(args.model, conf=args.conf)
    cam_pts, rob_pts, oris = [], [], []
    print("\n=== 손-눈 캘리브레이션 시작 ===")
    print("수동조작이 필요하면 다른 창에서: mc.release_all_servos() / 끝나면 mc.focus_all_servos()")

    try:
        while True:
            cmd = input(f"\n[{len(cam_pts)}점 수집됨] 새 점 측정=Enter / 저장=s / 종료=q : ").strip().lower()
            if cmd == "q":
                break
            if cmd == "s":
                if len(cam_pts) < 4:
                    print("  최소 4점 필요합니다."); continue
                R, t, rms = solve_rigid_transform(cam_pts, rob_pts)
                data = {"R": R.tolist(), "t": t.tolist(),
                        "grasp_ori": list(np.mean(oris, axis=0)),  # 평균 그리퍼 자세
                        "rms_mm": rms, "n_points": len(cam_pts)}
                with open(args.out, "w") as f:
                    json.dump(data, f, indent=2)
                print(f"  저장 완료: {args.out}  (잔차 RMS={rms:.1f}mm, {len(cam_pts)}점)")
                if rms > 15:
                    print("  ⚠ RMS가 큼(>15mm). 점을 더 넓게/정확히 다시 측정 권장")
                continue

            # 1) 카메라로 토마토 3D 잡기 (몇 프레임 평균)
            samples = []
            for _ in range(10):
                _, dets = cam.read()
                b = cam.best(dets, by="conf")
                if b:
                    samples.append(b["cam_xyz"])
            if not samples:
                print("  토마토를 못 봤습니다. 위치/조명 확인 후 다시."); continue
            cam_xyz = np.median(np.array(samples), axis=0)
            print(f"  카메라 좌표(m): X={cam_xyz[0]:+.3f} Y={cam_xyz[1]:+.3f} Z={cam_xyz[2]:+.3f}")

            # 2) 로봇팔 끝을 같은 토마토에 댄 뒤 Enter
            input("  → 그리퍼 끝을 '같은 토마토 중심'에 대고 Enter (움직임 멈춘 상태에서)")
            c = read_coords(mc)
            if not c:
                print("  로봇 좌표 읽기 실패. 다시."); continue
            print(f"  로봇 좌표(mm): X={c[0]:.1f} Y={c[1]:.1f} Z={c[2]:.1f}  자세={c[3:]} ")

            cam_pts.append(cam_xyz * 1000.0)   # m → mm
            rob_pts.append(c[:3])
            oris.append(c[3:])
    except KeyboardInterrupt:
        pass
    finally:
        cam.stop()
    print("종료.")


if __name__ == "__main__":
    main()
