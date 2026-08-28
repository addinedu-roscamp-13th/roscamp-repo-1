#!/usr/bin/env python3
"""잔차(파지 오프셋) 보정 — B캘리브 + A방식 마무리 (5분)
====================================================
문제: 픽 명령의 기준점이 손목(플랜지)이라, 토마토 좌표로 명령하면
     손가락이 책상을 파고듦(2026-07-03 영상 사고의 근본 원인).
해법: "계산된 토마토 위치" vs "손끝이 토마토를 감싸는 순간의 손목좌표"의
     차이(오프셋 벡터)를 2~3회 실측 → handeye_full.json 에 grasp_offset 저장.
     → auto_scan_pick 이 자동 적용 = 정확한 XYZ 도착 후 꽉 쥐기.

절차(토마토 1개당):
  ① 토마토를 책상에 놓고 Enter → 카메라가 위치 계산
  ② f 입력(힘 풀림·팔 잡기!) → 그리퍼를 "집는 자세 그대로"
     (손가락이 토마토 좌우를 감싸고, 손끝 높이=토마토 중심) 에 대고 Enter
  ③ l 입력(잠금) → 토마토 옮겨서 반복. s = 평균 저장.

실행: source /opt/ros/jazzy/setup.bash; export ROS_DOMAIN_ID=20
      export PYTHONPATH="$PYTHONPATH:$HOME/sam3/.venv/lib/python3.12/site-packages"
      python3 calibrate_grasp_offset.py
"""
import json
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from calibrate_handeye_full import cam_point_to_base, PVIEW

HE_PATH = "handeye_full.json"


class ArmLink(Node):
    def __init__(self):
        super().__init__('grasp_offset_calib')
        self.pub = self.create_publisher(String, '/automato/manual_cmd', 10)
        self.create_subscription(String, '/automato/arm_coords', self._on, 10)
        self.coords, self.t = None, 0.0

    def _on(self, msg):
        try:
            d = json.loads(msg.data); c = d.get("coords")
            if c and len(c) == 6:
                self.coords, self.t = c, time.time()
        except Exception:
            pass

    def cmd(self, c):
        m = String(); m.data = c
        self.pub.publish(m)

    def fresh(self, timeout=3.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            if self.coords and time.time() - self.t < 2.0:
                return list(self.coords)
            time.sleep(0.1)
        return None


def main():
    from td435_common import TomatoD435
    he = json.load(open(HE_PATH))
    rclpy.init()
    arm = ArmLink()
    threading.Thread(target=lambda: rclpy.spin(arm), daemon=True).start()
    if not arm.fresh(10):
        print("⚠ 팔 좌표 미수신 — dg_control_node/도메인 확인"); return
    while arm.pub.get_subscription_count() == 0:
        time.sleep(0.2)
    print("✅ 로봇 연결. 관찰자세로 이동…")
    arm.cmd("angles:" + json.dumps(PVIEW)); time.sleep(6)

    cam = TomatoD435("cherry_tomato.pt", conf=0.5)
    offsets = []
    print("\n=== 파지 오프셋 보정 ===")
    print("명령: Enter=토마토 측정  f=힘풀기  l=잠금  s=평균 저장  q=종료")
    try:
        while True:
            c = input("\n[%d회 측정됨] > " % len(offsets)).strip().lower()
            if c == "q":
                break
            if c == "f":
                arm.cmd("free"); print("  🔓 풀림 — 팔 잡으세요! 집는 자세로 토마토를 감싸고 Enter"); continue
            if c == "l":
                arm.cmd("lock"); print("  🔒 잠금"); continue
            if c == "s":
                if not offsets:
                    print("  측정값 없음"); continue
                off = np.mean(np.array(offsets), axis=0)
                he["grasp_offset"] = [round(float(v), 1) for v in off]
                he["grasp_offset_n"] = len(offsets)
                json.dump(he, open(HE_PATH, "w"), indent=2)
                print("  💾 저장: grasp_offset =", he["grasp_offset"],
                      "(측정 %d회 평균) → %s" % (len(offsets), HE_PATH))
                continue

            # ① 카메라로 토마토 위치 계산
            samples = []
            for _ in range(10):
                _, dets = cam.read()
                b = cam.best(dets, by="near")
                if b:
                    samples.append(b["cam_xyz"])
            if not samples:
                print("  토마토 안 보임 — 위치 확인"); continue
            cur = arm.fresh()
            if not cur:
                print("  팔좌표 미수신"); continue
            calc = cam_point_to_base(np.median(np.array(samples), axis=0) * 1000.0, cur, he)
            print("  계산된 토마토 위치: (%.1f, %.1f, %.1f)" % tuple(calc))
            # ② 사용자가 집는 자세로 대준 뒤 Enter
            input("  → f로 풀고, '집는 자세'로 토마토를 감싼 뒤(움직임 멈춤) Enter ")
            now = arm.fresh()
            if not now:
                print("  ⚠ 좌표 미수신 — 이 측정 버림"); continue
            off = [now[0]-calc[0], now[1]-calc[1], now[2]-calc[2]]
            offsets.append(off)
            print("  실측 손목좌표: (%.1f, %.1f, %.1f) → 오프셋 (%.1f, %.1f, %.1f)"
                  % (now[0], now[1], now[2], *off))
            print("  (z 오프셋이 +5~12cm 나오는 게 정상 — 손목·손끝 거리)")
    except KeyboardInterrupt:
        pass
    finally:
        cam.stop()
        arm.cmd("lock")
        rclpy.shutdown()
    print("종료.")


if __name__ == "__main__":
    main()
