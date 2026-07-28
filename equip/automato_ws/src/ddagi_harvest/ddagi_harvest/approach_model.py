#!/usr/bin/env python3
"""위치별 접근 모델 — 손 티칭한 접근 자세(taught_approaches.json)를 토마토 lateral
위치(base y)로 보간해, 각 토마토의 pre-grasp·grasp flange 자세를 만든다.

왜: 고정 그리퍼 자세 하나로는 두 줄·끝 토마토를 못 딴다(끝은 바깥에서 접근해야
줄기를 안 건다). 각 위치의 '이상적 접근'을 사람이 시연 → 그 사이를 보간. 나중에 AI가
접근 벡터를 주면 이 모델을 대체(또는 폴백). 시연은 tf_verify 'a' 로 쌓는다.

각 레코드: {"base":[x,y,z], "pregrasp":[x,y,z,rx,ry,rz], "grasp":[...6...]}  (mm/deg)
반환은 flange 목표(6). base는 tf 결과(손끝 목표)이고, 티칭 grasp는 flange 실측이라
오프셋(grasp-base)에 손끝→flange(TCP)·처짐까지 이미 포함돼 있다 → pick은 그대로 send.

선택: 3D로 **가장 가까운 시연점 하나**를 골라, 그 시연의 (오프셋+자세)를 통째로 적용.
왜 보간이 아니라 최근접이냐 — 각 시연은 '오프셋 ↔ 자세'가 정합된 한 쌍(그 자세라야
그 오프셋으로 손끝이 토마토에 닿음). 오프셋·자세를 따로 선형보간하면 자세의 비선형성
때문에 둘이 어긋나 손끝이 빗나간다(실측 확인). 최근접은 정합된 쌍을 그대로 쓰므로,
가까운 토마토엔 base 차이만큼만 평행이동돼 손끝이 제대로 떨어진다(시연 자체 오차만 남음).
시연을 촘촘히 쌓을수록 정확해진다.
"""
from __future__ import annotations

import json
import os

# tf_verify 가 저장하는 위치(패키지 루트)와 동일하게 잡는다.
_DEFAULT_PATH = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "taught_approaches.json")


def load(path: str | None = None) -> list:
    """시연 레코드 로드. 없거나 손상되면 빈 리스트(→ pick 은 고정 오프셋 폴백)."""
    try:
        with open(path or _DEFAULT_PATH) as f:
            data = json.load(f)
    except Exception:
        return []
    return [r for r in data
            if all(k in r for k in ("base", "pregrasp", "grasp"))
            and len(r["base"]) >= 3 and len(r["pregrasp"]) >= 6
            and len(r["grasp"]) >= 6]


def _dist2(base, rec) -> float:
    """토마토 base와 시연 base의 3D 제곱거리."""
    return sum((base[j] - rec["base"][j]) ** 2 for j in range(3))


def _apply(rec: dict, base, key: str) -> list:
    """시연 rec 의 key('pregrasp'/'grasp')를 이 base 기준 flange 목표(6)로.
    오프셋(rec[key]-rec[base])을 이 base 에 얹고, 자세(rx,ry,rz)는 그대로 승계."""
    return ([base[j] + (rec[key][j] - rec["base"][j]) for j in range(3)]
            + list(rec[key][3:6]))


# 이 거리(mm) 안에 시연점이 있을 때만 taught 접근을 쓴다. 없으면 None → pick 이
# 고정 접근으로 폴백. '문제 구역(오른쪽 끝 등)만 국소 티칭'하고 나머지는 고정 접근 유지.
LOCAL_RADIUS_MM = 45.0


def plan(base, taught: list | None = None, max_dist: float = LOCAL_RADIUS_MM):
    """base[x,y,z] → (pregrasp_flange6, grasp_flange6). 가까운 시연 없으면 None.

    3D 최근접 시연점의 (오프셋+자세)를 정합된 한 쌍으로 적용한다. 단, 최근접 시연이
    max_dist 보다 멀면 None(그 위치엔 시연이 없다고 보고 고정 접근으로 폴백).
    """
    pts = taught if taught is not None else load()
    if not pts:
        return None
    rec = min(pts, key=lambda r: _dist2(base, r))
    if _dist2(base, rec) > max_dist * max_dist:
        return None
    return _apply(rec, base, "pregrasp"), _apply(rec, base, "grasp")


if __name__ == "__main__":
    # 실 시연 데이터로 보간 확인 (하드웨어 불필요)
    pts = load()
    print(f"시연점 {len(pts)}개 로드")
    if pts:
        ys = sorted(r["base"][1] for r in pts)
        print(f"  base y 범위: {ys}")
        for by in (ys[0], (ys[0] + ys[-1]) / 2, ys[-1], -3.5, 115.7):
            pl = plan([240.0, by, 300.0])
            if pl:
                pre, gr = pl
                print(f"  y={by:7.1f} → grasp자세={[round(a, 1) for a in gr[3:6]]}"
                      f"  grasp위치={[round(c, 1) for c in gr[:3]]}")
