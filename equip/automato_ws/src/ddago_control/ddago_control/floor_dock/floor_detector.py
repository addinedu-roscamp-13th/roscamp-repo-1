# -*- coding: utf-8 -*-
"""바닥 H 마커 검출 + 바닥 좌표 변환 (순수 모듈, ROS 비의존).

RP-126. floor_calib.npz(바닥 평면, 내장 mtx/dist)를 이용해:
  - 청색 H 테이프를 마스크로 검출
  - 픽셀 -> 바닥 좌표(m) 변환 (ray-평면 교차, 벡터화)
  - minAreaRect 로 다리축·가로바 중심 산출 (마커리스)

`floor_dock_ws/floor_square.py`(ddago01 실주행 검증본)에서 검출부만 이식.
검출 튜닝 상수는 모듈 레벨(조명/카메라에 맞춰 조정). 도킹 기하·캘리브 경로는
노드 파라미터(floor_dock_server)가 관장한다.
"""
import math

import cv2
import numpy as np

# 도킹 마커 모양: "H"(마커리스 H자, 근거리 강건) / "rect"(사각형 테두리, 폴백)
SHAPE = "H"
TAPE = "blue"              # "blue": 청색 HSV 검출 / "dark": 명도 검출(폴백)

# ---- H 마커 규격/판별 (실측: 다리240·외측폭180·내측130·가로바 벽서30, 청색25mm) ----
H_MIN_AREA_PX = 1200       # 파란 덩어리 최소 픽셀면적(오검출 제거)
H_MIN_PTS = 60             # 바닥투영 유효점 최소
H_TBINS = 20               # 다리축(전방) 히스토그램 bin 수(가로바 위치 탐색)
H_MAX_SOLIDITY = 0.62      # solidity(면적/볼록껍질) 상한. H~0.4, 바퀴/원반~0.9+ 배제
H_LEN_RANGE = (0.08, 0.32)  # 다리축(전방) 길이[m] 허용범위(근거리 클리핑 대비 하한 여유)
H_WID_RANGE = (0.14, 0.22)  # 횡(가로바 폭) 길이[m]. 하한0.14=한쪽 다리만(측면클리핑) 거부
# 청색 HSV 범위 (OpenCV H:0-180). 조명/바닥/카메라에 따라 조정.
BLUE_LO = (90, 60, 40)
BLUE_HI = (135, 255, 255)

# ---- 사각형(rect) 폴백 규격 ----
SIDE_MIN = 0.14
SIDE_MAX = 0.26
SIDE_RATIO_TOL = 0.30


class FloorMapper:
    """floor_calib.npz(바닥 평면 + 내장 mtx/dist)로 픽셀↔바닥 좌표 변환."""

    def __init__(self, calib_path="floor_calib.npz"):
        d = np.load(calib_path)
        self.mtx, self.dist = d["mtx"], d["dist"]
        self.n, self.c = d["n"], float(d["c"])
        self.O, self.right, self.forward = d["O"], d["right"], d["forward"]

    def pixel_to_ground(self, pts):
        """(N,2) 픽셀 -> (N,2) 바닥 좌표 [m] (x=오른쪽, y=전방). 뒤쪽/수평선 위는 NaN.

        벡터화(H 마커는 수백~수천 점을 매 프레임 변환하므로 파이썬 루프 금지)."""
        pts = np.asarray(pts, np.float32).reshape(-1, 1, 2)
        rays = cv2.undistortPoints(pts, self.mtx, self.dist).reshape(-1, 2)
        n = np.asarray(self.n, float).ravel()
        r = np.column_stack([rays, np.ones(len(rays))])      # (N,3)
        denom = r @ n                                         # (N,)
        out = np.full((len(r), 2), np.nan)
        s = np.divide(self.c, denom, out=np.full_like(denom, np.nan),
                      where=np.abs(denom) > 1e-9)             # (N,)
        good = np.isfinite(s) & (s > 0)                       # 카메라 앞쪽만
        X = s[:, None] * r - np.asarray(self.O, float).ravel()  # (N,3)
        out[good, 0] = (X @ np.asarray(self.right, float).ravel())[good]
        out[good, 1] = (X @ np.asarray(self.forward, float).ravel())[good]
        return out


def tape_mask(img):
    """테이프(청색/어두운색) 전경 마스크(테이프=255). img 는 BGR 또는 그레이."""
    if TAPE == "blue":
        bgr = img if img.ndim == 3 else cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array(BLUE_LO, np.uint8),
                           np.array(BLUE_HI, np.uint8))
    else:
        gray = img if img.ndim == 2 else cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        mask = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                     cv2.THRESH_BINARY_INV, 51, 10)
    return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))


def _h_ground_geom(g):
    """바닥투영점 g(N,2)로부터 (center, heading, leg_len, width) 산출. 부족하면 None.

    ⚠️ PCA(분산)는 다리가 좌우로 벌어지면 횡분산≈종분산이라 고유벡터가 불안정(대각회전).
       → 밀도에 둔감한 **minAreaRect** 로 축을 잡고, 두 변 중 '로봇->H 방향'에 가까운
       쪽을 다리(접근)축으로(근접 클리핑으로 폭>길이여도 안전). 가로바=다리축 밀도피크."""
    if len(g) < H_MIN_PTS:
        return None
    box = cv2.boxPoints(cv2.minAreaRect(g.astype(np.float32)))
    ea, eb = box[1] - box[0], box[2] - box[1]
    la, lb = float(np.linalg.norm(ea)), float(np.linalg.norm(eb))
    if la < 1e-6 or lb < 1e-6:
        return None
    axa, axb = ea / la, eb / lb
    c0 = g.mean(0)
    bearing0 = math.atan2(float(c0[0]), float(c0[1]))

    def _axdiff(e):
        return abs((math.atan2(float(e[0]), float(e[1])) - bearing0 + math.pi / 2)
                   % math.pi - math.pi / 2)
    if _axdiff(axa) <= _axdiff(axb):                     # 다리(접근)축 = 로봇방향에 가까운 변
        e1, leg_len, width = axa, la, lb
    else:
        e1, leg_len, width = axb, lb, la
    t = (g - c0) @ e1
    tb = np.linspace(t.min(), t.max(), H_TBINS + 1)
    cnts_bin = np.array([np.sum((t >= tb[i]) & (t < tb[i + 1])) for i in range(H_TBINS)])
    if cnts_bin.max() == 0:
        return None
    k = int(np.argmax(cnts_bin))
    sel = (t >= tb[max(0, k - 1)]) & (t < tb[min(H_TBINS, k + 2)])    # 피크 ±1 bin
    center = g[sel].mean(0)                              # 가로바 중심(획 대칭이라 좌우 0)
    heading = math.atan2(float(e1[0]), float(e1[1]))     # 다리축 각(부호 무관)
    return center, float(heading), leg_len, width


def find_dock_h(img, mapper, prior_heading=0.0, roi_y=(0.1, 1.2), roi_x=0.6):
    """H 마커 검출 -> (center, heading, size, contour_px) 또는 None.

    center = 가로바 중심 바닥좌표(right,forward)[m] ← 도킹 목표. heading = 다리축.
    '가장 큰 파란 덩어리 = H' 가 아니라 모든 후보를 solidity·크기·ROI 로 검증(바퀴 배제)."""
    mask = tape_mask(img)
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best = None
    for cnt in cnts:
        area = cv2.contourArea(cnt)
        if area < H_MIN_AREA_PX:
            continue
        hull_area = cv2.contourArea(cv2.convexHull(cnt))
        if hull_area <= 0 or area / hull_area > H_MAX_SOLIDITY:   # 볼록(바퀴/원반) 배제
            continue
        comp = np.zeros(mask.shape, np.uint8)
        cv2.drawContours(comp, [cnt], -1, 255, -1)
        ys, xs = np.where((comp > 0) & (mask > 0))           # 이 컴포넌트 획 픽셀만
        if len(xs) < H_MIN_PTS:
            continue
        if len(xs) > 2000:                                   # 과점 시 서브샘플(Pi4 CPU)
            idx = np.linspace(0, len(xs) - 1, 2000).astype(int)
            xs, ys = xs[idx], ys[idx]
        g = mapper.pixel_to_ground(np.stack([xs, ys], 1).astype(np.float32))
        g = g[np.isfinite(g).all(1)]
        geom = _h_ground_geom(g)
        if geom is None:
            continue
        center, heading, leg_len, width = geom
        if not (H_LEN_RANGE[0] <= leg_len <= H_LEN_RANGE[1]):        # 크기 검증
            continue
        if not (H_WID_RANGE[0] <= width <= H_WID_RANGE[1]):
            continue
        if not (roi_y[0] < center[1] < roi_y[1] and abs(center[0]) < roi_x):
            continue
        if best is None or area > best[4]:                   # 유효 H 중 큰 것
            best = (center, heading, leg_len,
                    cnt.reshape(-1, 2).astype(np.float32), area)
    if best is None:
        return None
    return best[0], best[1], best[2], best[3]


def detect_square_px(img, min_area_px=1500):
    """(rect 폴백) 마스크 -> 윤곽 -> 볼록 4각형 후보 픽셀 코너 목록."""
    bw = tape_mask(img)
    contours, _ = cv2.findContours(bw, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    quads = []
    for cnt in contours:
        if cv2.contourArea(cnt) < min_area_px:
            continue
        approx = cv2.approxPolyDP(cnt, 0.03 * cv2.arcLength(cnt, True), True)
        if len(approx) == 4 and cv2.isContourConvex(approx):
            quads.append(approx.reshape(4, 2).astype(np.float32))
    return quads


def square_pose(ground_pts, prior_heading=0.0):
    """(rect 폴백) 코너 4개 -> (center, heading, side) 또는 None."""
    if np.isnan(ground_pts).any():
        return None
    sides = [np.linalg.norm(ground_pts[(i + 1) % 4] - ground_pts[i]) for i in range(4)]
    m = float(np.mean(sides))
    if not (SIDE_MIN <= m <= SIDE_MAX):
        return None
    if (max(sides) - min(sides)) / m > SIDE_RATIO_TOL:
        return None
    center = ground_pts.mean(axis=0)
    e = ground_pts[1] - ground_pts[0]
    h = math.atan2(e[0], e[1])
    cands = [(h + k * math.pi / 2 + math.pi) % (2 * math.pi) - math.pi for k in range(4)]
    heading = min(cands, key=lambda a: abs(a - prior_heading))
    return center, heading, m


def find_dock_square(img, mapper, prior_heading=0.0, roi_y=(0.1, 1.2), roi_x=0.6):
    """(rect 폴백) 사각형 검출 -> (center, heading, side, corners_px) 또는 None."""
    best = None
    for q in detect_square_px(img):
        g = mapper.pixel_to_ground(q)
        pose = square_pose(g, prior_heading)
        if pose is None:
            continue
        center, heading, side = pose
        if not (roi_y[0] < center[1] < roi_y[1] and abs(center[0]) < roi_x):
            continue
        if best is None or side > best[2]:
            best = (center, heading, side, q)
    return best


def find_dock(img, mapper, prior_heading=0.0):
    """SHAPE 에 따라 H 또는 사각형 검출. 반환 (center, heading, size, contour_px)."""
    if SHAPE == "H":
        return find_dock_h(img, mapper, prior_heading)
    return find_dock_square(img, mapper, prior_heading)
