#!/usr/bin/env python3
"""관측자세 camera→flange 변환 피팅 — 터치 실측 쌍으로 직접 구한다.

왜 이 방식인가
--------------
검출은 항상 **고정 관측자세**에서만 하므로 `FK(base←joint6) · X(joint6←camera)` 는
하나의 고정 변환이다. 그 변환을 캘리 값들로 조립하는 대신, **손끝이 열매에 닿는
flange 좌표**를 직접 측정해 피팅한다. 그러면 아래 오차가 **전부 한꺼번에 흡수**된다:

  · 핸드아이 캘리 오차 / 카메라 마운트 미세 이동
  · URDF FK 와 pymycobot send_coords 프레임 차이(실측 ~8mm)
  · TCP 오프셋(손끝-플랜지) 추정 오차
  · 서보 처짐의 계통 성분

측정값이 '우리가 실제로 명령하는 프레임(send_coords flange)'에 있기 때문에, 피팅
결과를 그대로 명령하면 된다 → TCP_CORRECTION·DESCEND_OFFSET·구역보정이 불필요.

남는 오차는 팔의 **반복정확도**(저가 팔의 물리적 한계)와 측정 잡음뿐이다.

수집: tf_verify 에서 'o' → 토마토 클릭 → 'g' → 조그로 손끝을 열매에 닿게 → 'x'
      (매번 observe_tf_pairs.json 에 누적. 베드 전체에 퍼뜨려 5~6점 이상)

실행:
    ~/venv/automato/bin/python ddagi_harvest/fit_observe_tf.py
    MODEL=affine ...   # 기본 rigid. affine 은 depth 스케일 오차까지 흡수(6점 이상 권장)
"""
from __future__ import annotations

import json
import os
import sys

import numpy as np

PAIRS_FILE = os.environ.get("PAIRS_FILE", "observe_tf_pairs.json")


def fit_rigid(P: np.ndarray, Q: np.ndarray):
    """P(camera) → Q(flange) 최적 회전+이동 (Kabsch). 스케일 고정."""
    cp, cq = P.mean(0), Q.mean(0)
    H = (P - cp).T @ (Q - cq)
    U, _, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    R = Vt.T @ np.diag([1, 1, d]) @ U.T
    return R, cq - R @ cp


def fit_affine(P: np.ndarray, Q: np.ndarray):
    """P → Q 최소자승 아핀(3x3 + 이동). 회전 외 스케일·전단까지 흡수."""
    A = np.hstack([P, np.ones((len(P), 1))])
    M, *_ = np.linalg.lstsq(A, Q, rcond=None)
    return M[:3].T, M[3]


def report(P, Q, R, t, label):
    pred = P @ R.T + t
    err = np.linalg.norm(pred - Q, axis=1)
    print(f"\n=== {label} ===")
    print(f"  잔차: 평균 {err.mean():.1f}mm  최대 {err.max():.1f}mm  "
          f"(점별: {[round(e, 1) for e in err]})")
    return err


def main() -> int:
    try:
        pairs = json.load(open(PAIRS_FILE))
    except FileNotFoundError:
        print(f"{PAIRS_FILE} 없음 — tf_verify 에서 'g'→'x' 로 측정을 모으세요")
        return 1
    P = np.array([p["camera"] for p in pairs], float)
    Q = np.array([p["flange"] for p in pairs], float)
    print(f"측정 쌍 {len(P)}개")
    for i, p in enumerate(pairs):
        print(f"  {i + 1}. camera={p['camera']} → flange={p['flange']}")
    if len(P) < 4:
        print("\n⚠ 최소 4점 필요(권장 6점 이상, 베드 전체에 퍼뜨려서). 더 모으세요.")
        return 1

    model = os.environ.get("MODEL", "rigid")
    R_r, t_r = fit_rigid(P, Q)
    e_r = report(P, Q, R_r, t_r, "rigid (회전+이동)")
    R_a, t_a = fit_affine(P, Q)
    e_a = report(P, Q, R_a, t_a, "affine (스케일·전단 포함)")

    R, t, name, err = ((R_a, t_a, "affine", e_a) if model == "affine"
                       else (R_r, t_r, "rigid", e_r))
    print(f"\n선택 모델: {name} (MODEL 환경변수로 변경)")
    if err.max() > 15:
        print("⚠ 최대 잔차 15mm 초과 — 측정 오류나 점 분포 부족일 수 있음."
              " 이상치 점을 지우거나 더 모으세요.")
    print("\n# tf_transform.py 에 붙여넣기 (내가 반영해줄게):")
    print("OBSERVE_CAM2FLANGE_R = [")
    for row in R:
        print(f"    [{row[0]:+.6f}, {row[1]:+.6f}, {row[2]:+.6f}],")
    print("]")
    print(f"OBSERVE_CAM2FLANGE_T = [{t[0]:+.2f}, {t[1]:+.2f}, {t[2]:+.2f}]")
    return 0


if __name__ == "__main__":
    sys.exit(main())
