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

# ── 스테이션 ID: 로마 숫자 인식(H 안 연두색 획) ──
#  goal.task_point_id 가 '1'~'3' 이면 그 번호 H(세로획 개수)만 채택(옆 스테이션 오검출 방지).
#  획은 청색 H 와 색이 달라(연두) 청색 검출 간섭 없음. GREEN_* 는 실측 테이프로 튜닝할 것.
#  현행 I/II/III(세로획 개수)만 = 3 스테이션. 향후 V/X 모양+좌우순서로 IV 이상 확장(C).
ROMAN_ENABLE = True
GREEN_LO = (35, 60, 60)          # 연두 HSV 하한 (OpenCV H 0~180)
GREEN_HI = (85, 255, 255)        # 연두 HSV 상한
ROMAN_MIN_AREA = 40              # 획 최소 픽셀면적(1차 노이즈 배제; 실측 크기는 아래로 판정)
ROMAN_THICK_MM = (10.0, 15.0)   # 획 두께 실측 범위 [mm] (바닥평면 투영)
ROMAN_LEN_MM = (43.0, 55.0)     # 획 길이 실측 범위 [mm]
ROMAN_MAX_ID = 3                 # 현행 인식 한계(I~III)


def station_id_from_point(task_point_id):
    """goal.task_point_id 가 '1'~'3'(로마숫자 I~III 스테이션)이면 그 번호, 아니면 0.
    0 = 로마 인식 게이트 미사용(어떤 H 나 도킹, 기존 동작)."""
    s = str(task_point_id).strip()
    if s.isdigit() and 1 <= int(s) <= ROMAN_MAX_ID:
        return int(s)
    return 0


def recognize_roman(frame, mapper, q):
    """H 윤곽/코너(q) 주변 연두색 로마 숫자 인식 → 규격 맞는 세로획 개수(I~III → ID 1~3).
    (ID숫자, boxes[(poly,두께mm,길이mm)], rejects[(poly,두께mm,길이mm)]) 반환. frame=오버레이 전 BGR.
    ★검출은 H 마커 내부(convex hull 을 pad 만큼 dilate)에서만 — 밖의 반사·타마커 잡검출 제거.
    (floor_dock_ws/floor_pose.recognize_roman 이식본, ddago02 실기 검증)."""
    x, y, w, h = cv2.boundingRect(q.astype(np.int32))
    pad = int(0.12 * max(w, h))     # H 경계 밖 여유(근접서 획 near부가 H 경계에 닿아 잘리는 것 방지)
    x0, y0 = max(0, x - pad), max(0, y - pad)
    x1 = min(frame.shape[1], x + w + pad)
    y1 = min(frame.shape[0], y + h + pad)
    roi = frame[y0:y1, x0:x1]
    if roi.size == 0:
        return 0, [], []
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array(GREEN_LO, np.uint8), np.array(GREEN_HI, np.uint8))
    # ★H 근처에서만★: H convex hull 을 pad 만큼 ★확장(dilate)★한 영역으로 제한.
    #  (erode 는 근접서 획 near부를 잘라먹었음. 파랑은 green∩ 로 이미 제외 → 확장해도 무해,
    #   대신 밖의 반사·타마커 잡검출은 여전히 H에서 멀어 배제.)
    hull = cv2.convexHull(q.astype(np.int32)) - np.array([[x0, y0]], np.int32)
    hmask = np.zeros(mask.shape, np.uint8)
    cv2.fillConvexPoly(hmask, hull, 255)
    k = max(3, pad)
    hmask = cv2.dilate(hmask, np.ones((k, k), np.uint8))
    mask = cv2.bitwise_and(mask, hmask)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    boxes = []            # 규격 통과 획: (poly(N,2 프레임픽셀), 두께mm, 길이mm) — 마름모 그대로
    rejects = []          # 크기 제한 밖: (poly(N,2), 두께mm, 길이mm)
    for c in cnts:
        if cv2.contourArea(c) < ROMAN_MIN_AREA:
            continue
        px = c.reshape(-1, 2).astype(np.float32) + np.array([x0, y0], np.float32)  # 프레임 픽셀 윤곽
        gp = mapper.pixel_to_ground(px)              # 바닥평면 [m]
        gp = gp[np.all(np.isfinite(gp), axis=1)]
        if len(gp) < 5:
            continue
        # ★바닥평면(실측)서 회전사각형★ → 원근/방향 무관한 실제 두께/길이(측정용).
        (_gc, (gw, gh), _ga) = cv2.minAreaRect(gp.astype(np.float32))
        short, lng = min(gw, gh) * 1000.0, max(gw, gh) * 1000.0      # [mm]
        # 표시용: minAreaRect(직각) 대신 ★실제 윤곽★(마름모/평행사변형 반영). approxPolyDP로 단순화.
        eps = 0.03 * cv2.arcLength(c, True)
        poly = cv2.approxPolyDP(c, eps, True).reshape(-1, 2).astype(np.float32) + np.array([x0, y0], np.float32)
        if (ROMAN_THICK_MM[0] <= short <= ROMAN_THICK_MM[1]
                and ROMAN_LEN_MM[0] <= lng <= ROMAN_LEN_MM[1]):
            boxes.append((poly, short, lng))
        else:
            rejects.append((poly, short, lng))
    boxes.sort(key=lambda b: float(b[0][:, 0].min()))   # 좌→우 정렬(번호 일관)
    return len(boxes), boxes, rejects


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
