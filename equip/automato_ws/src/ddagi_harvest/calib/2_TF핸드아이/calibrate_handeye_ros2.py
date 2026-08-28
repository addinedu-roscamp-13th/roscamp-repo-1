#!/usr/bin/env python3
"""
손-눈 캘리브레이션 — ROS2 다중자세(스캔) 버전 (SSH 불사용)
===========================================================
카메라가 그리퍼에 부착(eye-in-hand). 자동 스캔을 위해 관찰자세 5개
(scan_pose_1~5, J1 = -140/-70/+6/+76/+146도)를 쓰고, "자세마다" 전용
카메라→로봇 변환(R,t)을 만든다. 결과는 handeye_multi.json 하나에 저장.

  카메라(노트북 D435+YOLO)  : 로컬 직접 (td435_common.TomatoD435)
  로봇팔(라파이 dg_control) : ROS2 토픽만 사용
      명령 발행  /automato/manual_cmd  (scan_pose_N/free/lock/observe)
      좌표 구독  /automato/arm_coords  (2Hz)

자세 하나의 절차(자세당 4점 이상):
  ① pN 입력(팔이 스캔자세 N으로 이동) → ② 토마토 배치 → ③ Enter(카메라 캡처)
  → ④ f(서보 풀기, 팔 잡기!) → 그리퍼 끝을 토마토 중심에 → ⑤ Enter(로봇좌표 캡처)
  → ⑥ l(잠금) → 토마토 옮겨가며 ③~⑥ 반복 → 다음 자세는 다시 pN
  모든 자세가 4점 이상이면 s 로 저장.

실행(노트북):
  source /opt/ros/jazzy/setup.bash
  export ROS_DOMAIN_ID=20
  PYTHONPATH=~/sam3/.venv/lib/python3.12/site-packages \
      python3 calibrate_handeye_ros2.py --model tomato_4cls.pt
"""
import argparse
import json
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from td435_common import TomatoD435, solve_rigid_transform

N_POSES = 5
SCAN_J1 = [-140.0, -70.0, 6.24, 76.0, 146.0]   # 로봇 노드와 동일해야 함
FRESH_SEC = 2.0     # 팔 좌표가 이보다 오래됐으면 무효
MOVE_WAIT = 4.0     # 자세 이동 대기(초)


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--model", default="tomato_4cls.pt")
    p.add_argument("--out", default="handeye_multi.json")
    p.add_argument("--conf", type=float, default=0.4)
    return p.parse_args()


class ArmLink(Node):
    """dg_control_node 와의 ROS2 연결: 명령 발행 + 좌표 수신."""

    def __init__(self):
        super().__init__('handeye_calibrator')
        self.pub = self.create_publisher(String, '/automato/manual_cmd', 10)
        self.create_subscription(String, '/automato/arm_coords', self._on_coords, 10)
        self.coords = None
        self.coords_time = 0.0

    def _on_coords(self, msg):
        try:
            d = json.loads(msg.data)
            c = d.get("coords")
            if c and len(c) == 6:
                self.coords = c
                self.coords_time = time.time()
        except Exception:
            pass

    def cmd(self, c):
        m = String(); m.data = c
        self.pub.publish(m)
        self.get_logger().info('명령 발행: ' + c)

    def fresh_coords(self):
        if self.coords is None or time.time() - self.coords_time > FRESH_SEC:
            return None
        return list(self.coords)


def main():
    args = parse_args()
    rclpy.init()
    arm = ArmLink()
    threading.Thread(target=lambda: rclpy.spin(arm), daemon=True).start()

    print("로봇 좌표 수신 대기중… (dg_control_node 가 켜져 있어야 함)")
    for _ in range(40):
        if arm.fresh_coords():
            break
        time.sleep(0.25)
    if not arm.fresh_coords():
        print("⚠ /automato/arm_coords 미수신 — dg_control_node/ROS_DOMAIN_ID 확인!")
    else:
        print("✅ 로봇 좌표 수신 확인:", ["%.1f" % v for v in arm.fresh_coords()])

    cam = TomatoD435(args.model, conf=args.conf, ripe_only=False)
    # 자세별 수집함: data[k] = {"cam": [...], "rob": [...], "ori": [...]}
    data = {k: {"cam": [], "rob": [], "ori": []} for k in range(1, N_POSES + 1)}
    cur = 3   # 현재 자세(기본: 정면 3번)

    def counts():
        return " ".join("P%d:%d" % (k, len(data[k]["cam"])) for k in range(1, N_POSES + 1))

    print("\n=== 손-눈 캘리브레이션 (ROS2 · 스캔자세 %d개) ===" % N_POSES)
    print("명령: p1~p%d=자세 이동  Enter=새 점 측정  f=서보풀기  l=잠금  s=저장  q=종료" % N_POSES)
    print("⚠ pN 입력 시 팔이 크게 회전합니다 — 주변 비우고 진행!")

    try:
        while True:
            cmd = input(f"\n[{counts()}] (현재 P{cur}) > ").strip().lower()
            if cmd == "q":
                break
            if cmd in ("p1", "p2", "p3", "p4", "p5"):
                cur = int(cmd[1])
                arm.cmd("scan_pose_%d" % cur)
                print("  자세 P%d(J1=%.0f°)로 이동중… %d초 대기" % (cur, SCAN_J1[cur - 1], MOVE_WAIT))
                time.sleep(MOVE_WAIT)
                continue
            if cmd == "f":
                arm.cmd("free");  print("  서보 풀림 — 팔을 손으로 잡으세요!"); continue
            if cmd == "l":
                arm.cmd("lock");  print("  서보 잠금."); continue
            if cmd == "s":
                ready = [k for k in data if len(data[k]["cam"]) >= 4]
                if not ready:
                    print("  4점 이상 모인 자세가 없습니다."); continue
                out = {"scan_j1": SCAN_J1, "method": "ros2-multipose", "ts": time.time(), "poses": {}}
                for k in ready:
                    R, t, rms = solve_rigid_transform(data[k]["cam"], data[k]["rob"])
                    out["poses"][str(k)] = {
                        "j1_deg": SCAN_J1[k - 1],
                        "R": R.tolist(), "t": t.tolist(),
                        "grasp_ori": list(np.mean(data[k]["ori"], axis=0)),
                        "rms_mm": rms, "n_points": len(data[k]["cam"])}
                    flag = "⚠큼" if rms > 15 else "양호"
                    print("  P%d: RMS=%.1fmm (%d점) %s" % (k, rms, len(data[k]["cam"]), flag))
                missing = [k for k in data if k not in ready]
                if missing:
                    print("  (미완성 자세: %s — 나중에 이어서 하면 됨)" % missing)
                with open(args.out, "w") as f:
                    json.dump(out, f, indent=2)
                print("  저장 완료: %s (자세 %d개)" % (args.out, len(ready)))
                continue

            # ── 새 점 측정 (현재 자세 cur 기준) ──
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

            input("  → f로 풀고, 그리퍼 끝을 '같은 토마토 중심'에 댄 뒤 Enter ")
            c = arm.fresh_coords()
            if not c:
                print("  ⚠ 로봇 좌표 미수신(2초) — dg_control_node/도메인 확인. 이 점은 버림.")
                continue
            print(f"  로봇 좌표(mm): X={c[0]:.1f} Y={c[1]:.1f} Z={c[2]:.1f}")
            data[cur]["cam"].append(np.array(cam_xyz) * 1000.0)
            data[cur]["rob"].append(c[:3])
            data[cur]["ori"].append(c[3:])
            print("  P%d에 %d번째 점 저장. (잠그려면 l, 계속하려면 토마토 옮기고 Enter)"
                  % (cur, len(data[cur]["cam"])))
    except KeyboardInterrupt:
        pass
    finally:
        cam.stop()
        rclpy.shutdown()
    print("종료.")


if __name__ == "__main__":
    main()
