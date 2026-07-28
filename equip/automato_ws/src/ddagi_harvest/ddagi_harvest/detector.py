#!/usr/bin/env python3
"""토마토 검출 추상화 (DetectTomatoes) + Mock 구현.

수확 루프는 이 인터페이스(TomatoDetector.detect)만 호출한다. 지금은 실물 AI 서비스가
없으니 MockColorDetector(RealSense 색 검출)로 좌표·등급을 만들고, 나중에 민호님 AI
서비스가 오면 ServiceDetector 로 갈아끼운다 — detect() 시그니처는 동일.

detect() 반환: 토마토 dict 리스트
    {"base":[x,y,z]mm, "grade":"NORMAL"|"DISCARD", "uv":(u,v), "depth_cm":float}
base 는 tf_transform.camera_to_base_at_observe 결과라 **관측자세에서 호출**해야 맞다
(FK가 OBSERVE_ANGLES 가정). 루프가 검출 직전 move_observe 를 보장한다.

색→등급 매핑(Mock): 빨강·노랑=NORMAL(수확), 초록=DISCARD(미숙). 실물 AI는 자체 등급.
"""
from __future__ import annotations

import os
import sys
from abc import ABC, abstractmethod

import numpy as np

# 직접 실행(python3 ddagi_harvest/detector.py) 시 패키지 루트를 import 경로에 추가
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from ddagi_harvest import pick as pk       # noqa: E402
from ddagi_harvest import tf_transform as tf  # noqa: E402

W, H, FPS = 640, 480, 30

# HSV 색 범위 (OpenCV H:0~179). 실물 조명에서 미세조정.
_COLOR_RANGES = {
    "red": [((0, 100, 60), (10, 255, 255)), ((170, 100, 60), (179, 255, 255))],
    "yellow": [((18, 90, 80), (33, 255, 255))],
    "green": [((38, 60, 40), (85, 255, 255))],
}
_GRADE_BY_COLOR = {"red": "NORMAL", "yellow": "NORMAL", "green": "DISCARD"}
_MIN_AREA = 400          # 최소 블롭 픽셀 (너무 작은 잡티 제외)
_MIN_CIRCULARITY = 0.6   # 4πA/P² — 토마토는 둥글어 ~0.7↑, 잎·줄기는 길쭉해 낮음(오검출 컷)
_DEPTH_WIN = 5           # 중심 주변 (2n+1)^2 창의 depth 중앙값


def _open_realsense():
    """RealSense depth+color 스트림 시작 → (rs, pipe, align)."""
    import pyrealsense2 as rs
    pipe = rs.pipeline()
    cfg = rs.config()
    cfg.enable_stream(rs.stream.depth, W, H, rs.format.z16, FPS)
    cfg.enable_stream(rs.stream.color, W, H, rs.format.bgr8, FPS)
    pipe.start(cfg)
    return rs, pipe, rs.align(rs.stream.color)


def _median_depth(depth, u: int, v: int, win: int = _DEPTH_WIN) -> float:
    """(u,v) 주변 (2*win+1)^2 창의 유효 depth 중앙값(m). 없으면 0."""
    ds = []
    for dy in range(-win, win + 1):
        for dx in range(-win, win + 1):
            uu, vv = u + dx, v + dy
            if 0 <= uu < W and 0 <= vv < H:
                d = depth.get_distance(uu, vv)
                if d > 0:
                    ds.append(d)
    if not ds:
        return 0.0
    ds.sort()
    return ds[len(ds) // 2]


class TomatoDetector(ABC):
    @abstractmethod
    def detect(self) -> list[dict]:
        """현재 관측 시점의 토마토 리스트. 관측자세에서 호출해야 base 가 맞다."""

    def close(self) -> None:
        pass


class MockColorDetector(TomatoDetector):
    """RealSense 색 검출 기반 Mock — 실물 AI 대체. 노트북 카메라로 검출.

    include_discard=False 면 초록(미숙)은 검출에서 제외(익은 것만 수확).
    """

    def __init__(self, include_discard: bool = False, min_area: int = _MIN_AREA):
        self.include_discard = include_discard
        self.min_area = min_area
        self._rs, self.pipe, self.align = _open_realsense()

    def detect(self) -> list[dict]:
        import cv2
        frames = self.align.process(self.pipe.wait_for_frames())
        depth, color = frames.get_depth_frame(), frames.get_color_frame()
        if not depth or not color:
            return []
        intr = color.profile.as_video_stream_profile().get_intrinsics()
        img = np.asanyarray(color.get_data())
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        out: list[dict] = []
        for cname, ranges in _COLOR_RANGES.items():
            grade = _GRADE_BY_COLOR[cname]
            if grade == "DISCARD" and not self.include_discard:
                continue
            mask = None
            for lo, hi in ranges:
                m = cv2.inRange(hsv, np.array(lo), np.array(hi))
                mask = m if mask is None else (mask | m)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                           cv2.CHAIN_APPROX_SIMPLE)
            for c in contours:
                area = cv2.contourArea(c)
                if area < self.min_area:
                    continue
                perim = cv2.arcLength(c, True)
                if perim == 0:
                    continue
                circularity = 4 * np.pi * area / (perim * perim)
                if circularity < _MIN_CIRCULARITY:   # 잎·줄기 등 길쭉한 것 제외
                    continue
                mm = cv2.moments(c)
                if mm["m00"] == 0:
                    continue
                u, v = int(mm["m10"] / mm["m00"]), int(mm["m01"] / mm["m00"])
                d = _median_depth(depth, u, v)
                if d == 0:
                    continue
                cam = [c_ * 1000 for c_ in
                       self._rs.rs2_deproject_pixel_to_point(intr, [u, v], d)]
                base = [float(x) for x in tf.camera_to_base_at_observe(cam)]
                if not pk.in_workspace(base):
                    continue
                out.append({"base": base, "grade": grade, "uv": (u, v),
                            "depth_cm": d * 100, "color": cname})
        return out

    def close(self) -> None:
        try:
            self.pipe.stop()
        except Exception:
            pass


class YoloDetector(TomatoDetector):
    """실물 YOLO(.pt) 검출 — 김동현 학습모델. MockColorDetector 대체.

    클래스→동작 매핑: 익음=수확/NORMAL, 썩음·병해충=수확/DISCARD(폐기), 안익음=스킵.
    model.names 의 실제 클래스명에 맞춰 CLASS_MAP 조정(한/영 변형 미리 포함). 매핑에
    없는 클래스는 안전하게 스킵. 바운딩박스 중심을 depth 역투영해 base 좌표를 만든다.

    weights : .pt 경로 (네 노트북 로컬). 데이터셋 말고 weights/best.pt 하나면 됨.
    """

    # 실제 model.names 확인 후 필요시 수정. (harvest=따기, skip=안 따기)
    CLASS_MAP = {
        "익음": ("harvest", "NORMAL"), "ripe": ("harvest", "NORMAL"),
        "썩음": ("harvest", "DISCARD"), "rotten": ("harvest", "DISCARD"),
        "병해충": ("harvest", "DISCARD"), "pest": ("harvest", "DISCARD"),
        "disease": ("harvest", "DISCARD"), "diseased": ("harvest", "DISCARD"),
        "안익음": ("skip", None), "unripe": ("skip", None),
    }
    _BOX_DEPTH_FRAC = 0.25   # depth 샘플 창을 박스 절반폭의 이 비율로(중앙부만)

    def __init__(self, weights: str, conf: float = 0.4,
                 class_map: dict | None = None):
        from ultralytics import YOLO
        self.model = YOLO(weights)
        self.conf = conf
        self.class_map = class_map or self.CLASS_MAP
        self._rs, self.pipe, self.align = _open_realsense()
        print(f"[YoloDetector] 로드: {weights}")
        print(f"  모델 클래스: {self.model.names}")
        unmapped = [n for n in self.model.names.values() if n not in self.class_map]
        if unmapped:
            print(f"  ⚠ 매핑 없는 클래스(스킵됨): {unmapped} — CLASS_MAP 확인")

    def detect(self) -> list[dict]:
        frames = self.align.process(self.pipe.wait_for_frames())
        depth, color = frames.get_depth_frame(), frames.get_color_frame()
        if not depth or not color:
            return []
        intr = color.profile.as_video_stream_profile().get_intrinsics()
        img = np.asanyarray(color.get_data())

        res = self.model(img, conf=self.conf, verbose=False)[0]
        out: list[dict] = []
        for box in res.boxes:
            cls_name = self.model.names[int(box.cls[0])]
            action, grade = self.class_map.get(cls_name, ("skip", None))
            if action != "harvest":
                continue
            x1, y1, x2, y2 = (float(v) for v in box.xyxy[0])
            u, v = int((x1 + x2) / 2), int((y1 + y2) / 2)
            win = max(2, int(min(x2 - x1, y2 - y1) * self._BOX_DEPTH_FRAC))
            d = _median_depth(depth, u, v, win)
            if d == 0:
                continue
            cam = [c_ * 1000 for c_ in
                   self._rs.rs2_deproject_pixel_to_point(intr, [u, v], d)]
            base = [float(x) for x in tf.camera_to_base_at_observe(cam)]
            if not pk.in_workspace(base):
                continue
            out.append({"base": base, "grade": grade, "uv": (u, v),
                        "depth_cm": d * 100, "color": cls_name,
                        "conf": float(box.conf[0])})
        return out

    def close(self) -> None:
        try:
            self.pipe.stop()
        except Exception:
            pass


class ListDetector(TomatoDetector):
    """고정 리스트 반환(테스트/데모용). detect()마다 같은 목록을 준다."""

    def __init__(self, tomatoes: list[dict]):
        self._toms = tomatoes

    def detect(self) -> list[dict]:
        return [dict(t) for t in self._toms]


if __name__ == "__main__":
    # 검출만 확인(파지 없음). 관측자세로 옮긴 뒤 몇 프레임 검출해 출력.
    import os
    import sys
    import time
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    from ddagi_harvest.arm_backend import NetworkArm

    ip = os.environ.get("ARM_IP", "192.168.3.12")
    weights = os.environ.get("WEIGHTS", "")
    arm = NetworkArm(ip)
    print(f"팔 연결 {ip} — 관측자세로 이동")
    arm.move_angles(pk.OBSERVE_ANGLES, 30)
    time.sleep(1.0)
    det = YoloDetector(weights) if weights else MockColorDetector()
    try:
        toms = det.detect()
        print(f"\n검출 {len(toms)}개:")
        for i, t in enumerate(sorted(toms, key=lambda t: t["base"][1])):
            print(f"  {i:>2} {t['color']:>6}/{t['grade']:<7} "
                  f"base={[round(c, 1) for c in t['base']]} "
                  f"uv={t['uv']} depth={t['depth_cm']:.1f}cm")
    finally:
        det.close()
        arm.close()
