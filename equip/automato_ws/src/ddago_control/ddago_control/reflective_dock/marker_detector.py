"""
반사마커 검출 파이프라인 — 순수 로직 (ROS 의존 없음).

여기 있는 함수들은 라이다 '데이터'만 받아서 계산만 한다.
ROS 구독/발행/로그는 전부 detector_node.py 가 담당한다.
→ 이렇게 나눠야 로봇 없이도 이 로직을 테스트할 수 있다.

좌표계: 라이다(rplidar_link) 기준. 0°축은 로봇 꽁무니 방향.
        (지금은 검출 기하만 다루므로 이 프레임 그대로 쓴다.
         base_footprint 로의 TF 변환은 나중 단계에서 붙인다.)
"""
import math
from collections import namedtuple

# 점 하나 = 각도(도), 거리(m), 밝기, 그리고 변환된 x,y
Point = namedtuple("Point", ["angle_deg", "range_m", "intensity", "x", "y"])


def polar_to_xy(angle_deg, range_m):
    """S1: 극좌표(각도, 거리) → 직교좌표(x, y).

    라이다는 '몇 도 방향에 몇 m'로 점을 준다(극좌표).
    클러스터링·직선맞춤을 하려면 평면 위 (x, y)가 편하다.
        x = r·cos(θ),  y = r·sin(θ)
    """
    theta = math.radians(angle_deg)
    x = range_m * math.cos(theta)
    y = range_m * math.sin(theta)
    return x, y


def build_points(readings):
    """(angle_deg, range_m, intensity) 목록 → Point 목록 (x,y 채워서).

    S1을 모든 점에 적용해 주는 헬퍼.
    """
    points = []
    for angle_deg, range_m, intensity in readings:
        x, y = polar_to_xy(angle_deg, range_m)
        points.append(Point(angle_deg, range_m, intensity, x, y))
    return points


def filter_range(points, r_min, r_max):
    """S2: r_min < 거리 < r_max 인 점만 남긴다.

    - r_min 아래: 라이다 지지봉/브래킷 같은 로봇 자체 구조물(실측 0.05~0.08m) 제거
    - r_max 위:   멀리 있는 배경 벽 제거
    남는 건 '마커가 있을 법한 거리대'의 점들뿐.
    """
    return [p for p in points if r_min < p.range_m < r_max]


def _dist(a, b):
    """두 Point 사이의 실제 평면 거리(m)."""
    return math.hypot(a.x - b.x, a.y - b.y)


def cluster_by_gap(points, gap):
    """S3: 각도순 점들을 '인접 간격'으로 덩어리(클러스터)로 나눈다.

    라이다를 각도 순서로 훑으며, 이웃한 두 점의 실제 거리가 gap(기본 3cm)보다
    크면 거기서 끊어 서로 다른 물체로 본다. 그러면 마커·벽·잡음 조각이
    각각 별도 덩어리로 갈린다.

    ±180° 이음새 처리: 각도 목록의 처음(−180°쪽)과 끝(+180°쪽)은 사실
    같은 방향이라 물리적으로 이어져 있다. 정면에 놓인 마커가 딱 여기 걸린다.
    그래서 첫 점과 끝 점이 가까우면 두 끝 덩어리를 하나로 잇는다.
    """
    if not points:
        return []
    pts = sorted(points, key=lambda p: p.angle_deg)
    clusters = [[pts[0]]]
    for prev, cur in zip(pts, pts[1:]):
        if _dist(prev, cur) <= gap:
            clusters[-1].append(cur)
        else:
            clusters.append([cur])
    # 이음새에서 이어지면 마지막 덩어리를 첫 덩어리 앞에 붙인다
    if len(clusters) > 1 and _dist(pts[0], pts[-1]) <= gap:
        clusters[0] = clusters.pop() + clusters[0]
    return clusters


def _perp_dist(p, a, b):
    """점 p 에서 'a와 b를 잇는 직선'까지의 수직 거리(m).

    외적(cross product)의 크기를 두 점 사이 거리로 나누면 수직거리가 나온다.
    """
    dx, dy = b.x - a.x, b.y - a.y
    base = math.hypot(dx, dy)
    if base < 1e-9:  # a와 b가 같은 점이면
        return math.hypot(p.x - a.x, p.y - a.y)
    return abs((p.x - a.x) * dy - (p.y - a.y) * dx) / base


def split_iepf(segment, threshold):
    """S4: IEPF(=Douglas-Peucker)로 한 덩어리를 직선 세그먼트들로 나눈다.

    (1) 양 끝점을 잇는 직선을 긋는다.
    (2) 그 직선에서 제일 멀리 벗어난 중간 점을 찾는다.
    (3) 벗어난 거리가 threshold(기본 2cm)보다 크면 → 거기서 꺾인 것.
        그 점을 기준으로 둘로 쪼갠 뒤 각각 다시 반복(재귀).
        작으면 → 전체를 '직선 하나'로 인정.

    마커처럼 90°로 꺾인 덩어리는 꺾인 꼭짓점에서 두 직선으로 갈린다.
    반환: 세그먼트 목록 (각 세그먼트 = Point 목록)
    """
    if len(segment) < 2:
        return [segment] if segment else []
    a, b = segment[0], segment[-1]
    max_d, idx = -1.0, -1
    for i in range(1, len(segment) - 1):
        d = _perp_dist(segment[i], a, b)
        if d > max_d:
            max_d, idx = d, i
    if idx != -1 and max_d > threshold:
        # 꼭짓점(idx)에서 쪼개 재귀 (idx 점은 양쪽이 공유)
        left = split_iepf(segment[: idx + 1], threshold)
        right = split_iepf(segment[idx:], threshold)
        return left + right
    return [segment]  # 더 이상 안 꺾임 = 직선 하나


def segment_length(segment):
    """세그먼트(점 목록)의 길이 = 양 끝점 사이 거리(m). (S5에서 씀)"""
    if len(segment) < 2:
        return 0.0
    return _dist(segment[0], segment[-1])


def filter_length(segments, target, tol):
    """S5: 길이가 target±tol 인 세그먼트만 남긴다.

    마커 면은 15cm. 긴 벽(수십 cm)이나 짧은 잡음 조각을 여기서 버린다.
    """
    return [s for s in segments if abs(segment_length(s) - target) <= tol]


def filter_point_count(segments, min_points):
    """S6: 점 개수가 min_points 이상인 세그먼트만 남긴다.

    길이는 우연히 맞아도 점이 몇 개뿐인(성긴) 세그먼트는 신뢰 못 하므로 버린다.
    """
    return [s for s in segments if len(s) >= min_points]


def _segment_dir(segment):
    """세그먼트 방향 각도(rad): 첫 점 → 끝 점."""
    return math.atan2(segment[-1].y - segment[0].y, segment[-1].x - segment[0].x)


def angle_between(seg1, seg2):
    """두 세그먼트가 이루는 사잇각(도), 0~90 범위로 정규화.

    직선은 앞뒤 방향 구분이 없으므로(180° 모호성) 0~90 안으로 접어 넣는다.
    """
    d = abs(math.degrees(_segment_dir(seg1) - _segment_dir(seg2))) % 180.0
    return 180.0 - d if d > 90.0 else d


def _endpoints_touch(seg1, seg2, tol):
    """두 세그먼트의 끝점 중 tol(m) 이내로 맞닿는 게 있으면 True (=코너 공유)."""
    ends1 = (seg1[0], seg1[-1])
    ends2 = (seg2[0], seg2[-1])
    return any(_dist(a, b) <= tol for a in ends1 for b in ends2)


def find_right_angle_pairs(segments, angle_tol, endpoint_tol):
    """S7: 사잇각이 90°±angle_tol 이고 끝점이 endpoint_tol 이내로 맞닿는 쌍을 찾는다.

    마커는 '직각으로 만난 두 면'이므로 이 조건을 만족한다.
    반환: (i, j, 사잇각도) 목록
    """
    pairs = []
    for i in range(len(segments)):
        for j in range(i + 1, len(segments)):
            ang = angle_between(segments[i], segments[j])
            if abs(ang - 90.0) <= angle_tol and _endpoints_touch(
                segments[i], segments[j], endpoint_tol
            ):
                pairs.append((i, j, ang))
    return pairs


def _median(values):
    """중앙값. (평균보다 튐값에 강해서 밝기 판정에 쓴다)"""
    s = sorted(values)
    n = len(s)
    if n == 0:
        return 0.0
    mid = n // 2
    return s[mid] if n % 2 else (s[mid - 1] + s[mid]) / 2.0


def face_median_intensity(segment):
    """세그먼트(면)의 밝기 중앙값."""
    return _median([p.intensity for p in segment])


def classify_and_select(segments, pairs, reflective_min):
    """S8: 각 직각쌍에서 면 A/B를 가르고, 진짜 마커만 남긴다.

    - 두 면 중 밝기 중앙값이 높은 쪽 = 면 A(재귀반사·방향 기준),
      낮은 쪽 = 면 B(코드가 실린 면).
    - 면 A 중앙값이 reflective_min 이상이어야 '재귀반사 면을 가진 마커'로 인정.
      → 그냥 벽으로 된 직각(디코이)은 여기서 탈락.

    반환: 마커 후보 목록(면 A 밝을수록 앞). 각 원소는 dict.
    """
    markers = []
    for (i, j, ang) in pairs:
        s_i, s_j = segments[i], segments[j]
        m_i, m_j = face_median_intensity(s_i), face_median_intensity(s_j)
        if m_i >= m_j:
            face_a, face_b, a_med, b_med = s_i, s_j, m_i, m_j
        else:
            face_a, face_b, a_med, b_med = s_j, s_i, m_j, m_i
        if a_med >= reflective_min:
            markers.append(
                {
                    "face_a": face_a,
                    "face_b": face_b,
                    "a_median": a_med,
                    "b_median": b_med,
                    "angle": ang,
                }
            )
    markers.sort(key=lambda m: m["a_median"], reverse=True)
    return markers


def fit_line(segment):
    """세그먼트에 직교최소제곱(TLS)으로 직선을 맞춘다.

    y=mx+b 방식은 '수직에 가까운 선'에서 무한대로 터진다. 그래서
    무게중심을 지나며 점 분포의 '긴 축' 방향을 직선으로 삼는다(공분산 기반).
    반환: (cx, cy, angle_rad) = 무게중심, 직선 방향각.
    """
    n = len(segment)
    cx = sum(p.x for p in segment) / n
    cy = sum(p.y for p in segment) / n
    sxx = sum((p.x - cx) ** 2 for p in segment)
    syy = sum((p.y - cy) ** 2 for p in segment)
    sxy = sum((p.x - cx) * (p.y - cy) for p in segment)
    angle = 0.5 * math.atan2(2 * sxy, sxx - syy)
    return cx, cy, angle


def _intersect(line1, line2):
    """두 직선(각각 (cx,cy,angle))의 교점 (x,y). 평행이면 None."""
    c1x, c1y, a1 = line1
    c2x, c2y, a2 = line2
    d1x, d1y = math.cos(a1), math.sin(a1)
    d2x, d2y = math.cos(a2), math.sin(a2)
    det = -d1x * d2y + d2x * d1y
    if abs(det) < 1e-9:
        return None
    bx, by = c2x - c1x, c2y - c1y
    t = (-bx * d2y + d2x * by) / det
    return c1x + t * d1x, c1y + t * d1y


def _unit(x, y):
    n = math.hypot(x, y)
    return (x / n, y / n) if n > 1e-9 else (0.0, 0.0)


def _far_end(segment, vx, vy):
    """세그먼트의 두 끝점 중 (vx,vy)에서 더 먼 쪽 (꼭짓점 반대편 끝)."""
    e0, e1 = segment[0], segment[-1]
    d0 = math.hypot(e0.x - vx, e0.y - vy)
    d1 = math.hypot(e1.x - vx, e1.y - vy)
    return (e1.x, e1.y) if d1 > d0 else (e0.x, e0.y)


def _norm_ang(a):
    """각도를 -pi ~ pi 로 접는다(두 방향의 차이를 재려면 접어야 한다)."""
    return math.atan2(math.sin(a), math.cos(a))


def compute_pose(marker):
    """S11: 마커의 위치(원점)와 바라보는 방향(yaw)을 계산.

    - 원점: 두 면 직선의 '교점' = 코너 꼭짓점 (x, y)
    - yaw:  두 면이 벌어진 반대쪽 법선 = 재귀반사 면이 로봇을 향하는 방향.
            로봇은 이 축을 따라 진입(후진)한다.
    반환: dict(x, y, yaw_rad, id=None) / 계산불가면 None

    ⚠️ 마지막의 방향 검증이 왜 필요한가 (2026-08-03 실측):
      두 면 방향의 합으로 바깥쪽을 정하는 계산은 **정면에서 볼 때만 안정적**이다.
      비스듬히 보면 한 면이 짧게 잘려 보이고, 그러면 꼭짓점(두 직선의 교점)이
      반대편에 잡혀 두 면 방향이 함께 뒤집힌다 → yaw 가 정확히 180° 반대가 된다.
      실측: 마커 정면 축에서 6.6cm 옆에 선 상태에서 도킹하니, 로봇이 마커를 등지는
      대신 **마주 본 채** 후진 단계에 들어갔다(β=-179°).
      바로잡는 근거는 기하가 아니라 물리다 — 반사테이프는 충전소 벽에 붙어 있고
      라이다는 그 앞 공간에 있다. **라이다가 벽 속으로 들어갈 수는 없으므로**
      법선은 반드시 라이다(원점) 쪽을 향한다. 반대면 뒤집힌 것이다.
      원인이 무엇이든(면 잘림·꼭짓점 오판) 결과가 불가능하면 되돌리는 방식이라,
      검출 파이프라인 앞단을 건드리지 않고도 뒤집힘을 막는다.
    """
    la = fit_line(marker["face_a"])
    lb = fit_line(marker["face_b"])
    inter = _intersect(la, lb)
    if inter is None:
        return None
    vx, vy = inter
    ax, ay = _far_end(marker["face_a"], vx, vy)
    bx, by = _far_end(marker["face_b"], vx, vy)
    ua = _unit(ax - vx, ay - vy)  # 꼭짓점 → 면A 끝
    ub = _unit(bx - vx, by - vy)  # 꼭짓점 → 면B 끝
    # 두 면 방향의 합 = 코너가 '벌어지는(안쪽)' 방향. 바깥(로봇쪽) = 그 반대.
    facing = (-(ua[0] + ub[0]), -(ua[1] + ub[1]))
    yaw = math.atan2(facing[1], facing[0])
    # 라이다는 원점이므로 '마커 → 로봇' 은 곧 꼭짓점의 반대 방향이다.
    to_robot = math.atan2(-vy, -vx)
    if abs(_norm_ang(yaw - to_robot)) > math.pi / 2.0:
        yaw = _norm_ang(yaw + math.pi)
    return {"x": vx, "y": vy, "yaw_rad": yaw, "id": None}


def cell_medians(face_segment, n_bits):
    """면 B를 길이 방향으로 n등분해 각 칸의 밝기 중앙값을 구한다.

    각 점을 면 직선에 투영(내적)해 '면을 따라간 위치 t'를 얻고,
    t 범위를 n칸으로 쪼개 칸마다 밝기를 모아 중앙값을 낸다.
    반환: 길이 n_bits 의 중앙값 목록.
    """
    cx, cy, ang = fit_line(face_segment)
    dx, dy = math.cos(ang), math.sin(ang)
    ts = [((p.x - cx) * dx + (p.y - cy) * dy) for p in face_segment]
    tmin, tmax = min(ts), max(ts)
    span = tmax - tmin
    if span < 1e-6:
        return None
    cells = [[] for _ in range(n_bits)]
    for p, t in zip(face_segment, ts):
        k = min(int((t - tmin) / span * n_bits), n_bits - 1)
        cells[k].append(p.intensity)
    return [_median(c) if c else 0.0 for c in cells]


def read_code(face_segment, n_bits=3):
    """S9: 면 B의 각 칸 밝기로 1/0 비트를 읽어 코드를 만든다.

    문턱값 = '가장 어두운 칸과 밝은 칸의 중간'(상대비교, 절대값 아님).
    각 칸 중앙값이 문턱 이상이면 1(반사), 아니면 0(무반사).
    반환: 비트 튜플, 예) (0, 1, 0)
    """
    if len(face_segment) < n_bits:
        return None
    meds = cell_medians(face_segment, n_bits)
    if meds is None:
        return None
    thr = (min(meds) + max(meds)) / 2.0
    return tuple(1 if m >= thr else 0 for m in meds)


def run_pipeline(readings, cfg):
    """전체 검출 파이프라인 S1~S11 을 한 번에 실행한다 (노드가 호출).

    readings: (angle_deg, range_m, intensity) 목록
    cfg:      파라미터 dict
    반환: (검출된 마커 목록, 단계별 개수 dict)
          각 마커 = classify 결과 + pose(x,y,yaw_rad) + id
    """
    points = build_points(readings)
    ranged = filter_range(points, cfg["r_min"], cfg["r_max"])
    clusters = cluster_by_gap(ranged, cfg["gap"])
    segments = []
    for c in clusters:
        segments.extend(split_iepf(c, cfg["iepf"]))
    segs = filter_length(segments, cfg["seg_len"], cfg["seg_tol"])
    segs = filter_point_count(segs, cfg["min_points"])
    pairs = find_right_angle_pairs(segs, cfg["angle_tol"], cfg["endpoint_tol"])
    candidates = classify_and_select(segs, pairs, cfg["reflective_min"])

    markers = []
    for mk in candidates:
        pose = compute_pose(mk)
        if pose is None:
            continue
        pose["id"] = read_code(mk["face_b"], cfg["n_bits"])
        markers.append({**mk, **pose})

    counts = {
        "raw": len(points),
        "range": len(ranged),
        "clusters": len(clusters),
        "segments": len(segments),
        "len_filtered": len(segs),
        "pairs": len(pairs),
        "markers": len(markers),
    }
    return markers, counts
