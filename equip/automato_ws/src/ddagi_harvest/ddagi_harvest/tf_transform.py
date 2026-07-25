#!/usr/bin/env python3
"""camera_optical → base 좌표 변환 (Ddagi 수확 파이프라인의 기초).

AI(DetectTomatoes)가 준 토마토의 camera 좌표를, 로봇팔이 움직일 수 있는
base 좌표로 바꾼다. 파지 시퀀스·루프가 전부 이 변환 위에 선다.

변환 사슬 (건수님 캘리 3종, calib/ 폴더 원본):
    p_camera(camera_color_optical_frame)
      → [② 핸드아이] T(joint6 ← camera) 곱  → p_joint6
      → [get_coords] T(base ← joint6) 곱     → p_base
    ③ TCP(그리퍼 손끝 오프셋)는 '어디로 send_coords 할지' 계산 때 쓴다(파지 모듈).

핵심 사실:
- ② 핸드아이는 eye-in-hand라 **camera→joint6 는 자세 무관 고정값**. 그대로 재사용.
- T(base ← joint6)만 매 순간 달라진다 → 관측자세에서 get_coords()로 얻는다.
- ⚠️ 프레임 주의: 핸드아이는 camera_color_optical_frame 기준(z=앞). AI 출력도
  이 optical frame(rs2_deproject 원본)이어야 맞물린다. camera_link(x=앞)이면 90° 틀어짐.

단위: 전부 mm, 각도 deg (pymycobot get_coords 관례와 일치).
"""
from __future__ import annotations

import numpy as np

# --------------------------------------------------------------------------- #
# 건수님 캘리 결과 (2026-07 패키지, calib/ 폴더 원본). 검증 재투영 오차 3.4mm.
# 출처: calib/2_TF핸드아이/jetcobot_handeye.calib , calib/3_TCP캘리브/TCP_최종결과.txt
# --------------------------------------------------------------------------- #

# ② 핸드아이 eye-in-hand: T(joint6 ← camera_color_optical_frame)
#    .calib 의 translation(m) → mm 로, rotation 은 quaternion(x,y,z,w)
HANDEYE_TRANS_MM = np.array([-40.34080280, 28.87005975, 79.43500909])
HANDEYE_QUAT_XYZW = np.array([-0.74848690, -0.02614405, -0.01360042, 0.66249443])

# ③ TCP: 그리퍼 손끝 오프셋 (joint6/flange 기준, J6 무관), mm
TCP_FLANGE_MM = np.array([-3.2, -10.3, 109.2])

# pymycobot get_coords 의 rx,ry,rz 오일러 규약. 애매해서 실측 검증 필요.
# 팀 arm_tf_bridge 기본값과 동일하게 ZYX 로 둔다(=Rz@Ry@Rx).
DEFAULT_EULER_ORDER = "ZYX"


# ---- 회전/변환 기본 ---------------------------------------------------------- #

def quat_to_R(q_xyzw: np.ndarray) -> np.ndarray:
    """쿼터니언(x,y,z,w) → 3x3 회전행렬. 입력은 방어적으로 정규화한다."""
    x, y, z, w = q_xyzw / np.linalg.norm(q_xyzw)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
    ])


def _axis_R(axis: str, deg: float) -> np.ndarray:
    a = np.radians(deg)
    c, s = np.cos(a), np.sin(a)
    if axis == "X":
        return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
    if axis == "Y":
        return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])  # Z


def euler_to_R(rx: float, ry: float, rz: float,
               order: str = DEFAULT_EULER_ORDER) -> np.ndarray:
    """오일러각(deg) → 3x3 회전행렬. order 순서대로 곱한다(ZYX = Rz@Ry@Rx)."""
    ang = {"X": rx, "Y": ry, "Z": rz}
    R = np.eye(3)
    for ax in order:
        R = R @ _axis_R(ax, ang[ax])
    return R


def make_T(R: np.ndarray, t_mm: np.ndarray) -> np.ndarray:
    """3x3 회전 + 3 이동(mm) → 4x4 동차변환."""
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t_mm
    return T


# ---- 변환 사슬 -------------------------------------------------------------- #

def handeye_T() -> np.ndarray:
    """T(joint6 ← camera). 자세 무관 고정값."""
    return make_T(quat_to_R(HANDEYE_QUAT_XYZW), HANDEYE_TRANS_MM)


def base_from_flange_T(arm_coords, euler_order: str = DEFAULT_EULER_ORDER):
    """T(base ← joint6). 관측자세의 get_coords([x,y,z,rx,ry,rz], mm/deg)에서 만든다."""
    x, y, z, rx, ry, rz = arm_coords
    return make_T(euler_to_R(rx, ry, rz, euler_order), np.array([x, y, z]))


def camera_to_base(p_camera_mm, arm_coords,
                   euler_order: str = DEFAULT_EULER_ORDER) -> np.ndarray:
    """camera(optical) 좌표 → base 좌표 (mm).

    p_camera_mm : AI가 준 토마토 좌표 [x,y,z] (camera_color_optical_frame, mm)
    arm_coords  : 그 프레임을 찍을 때(관측자세)의 get_coords [x,y,z,rx,ry,rz]
    """
    p_cam = np.array([*p_camera_mm, 1.0])
    T = base_from_flange_T(arm_coords, euler_order) @ handeye_T()
    return (T @ p_cam)[:3]


def gripper_tip_offset_base(arm_coords,
                            euler_order: str = DEFAULT_EULER_ORDER) -> np.ndarray:
    """현재 자세에서 그리퍼 손끝이 flange보다 base 기준 얼마나 앞서는지(mm 벡터).

    파지 시 'send_coords 목표를 손끝 기준으로 보정'할 때 쓴다. (참고용)
    """
    R = euler_to_R(arm_coords[3], arm_coords[4], arm_coords[5], euler_order)
    return R @ TCP_FLANGE_MM


# ---- 자기검증 (하드웨어 없이) ------------------------------------------------- #

def _selftest() -> bool:
    ok = True

    # 1) 왕복 일관성: camera→base→camera 가 원점 복귀하는가
    arm = [150.0, -40.0, 300.0, -90.0, 0.0, -45.0]  # 임의 관측자세 예시
    p_cam = np.array([26.0, 81.2, 392.0])           # 건수님 실사용 예시 좌표
    p_base = camera_to_base(p_cam, arm)
    T = base_from_flange_T(arm) @ handeye_T()
    p_cam_back = (np.linalg.inv(T) @ np.array([*p_base, 1.0]))[:3]
    err = np.linalg.norm(p_cam - p_cam_back)
    print(f"[1] 왕복 오차: {err:.6f} mm  ({'OK' if err < 1e-6 else 'FAIL'})")
    ok = ok and err < 1e-6

    # 2) 핸드아이 정규직교성(회전행렬이 유효한가)
    R = quat_to_R(HANDEYE_QUAT_XYZW)
    orth = np.linalg.norm(R @ R.T - np.eye(3))
    det = np.linalg.det(R)
    print(f"[2] 핸드아이 R: 직교오차={orth:.2e}, det={det:.6f} "
          f"({'OK' if orth < 1e-9 and abs(det - 1) < 1e-9 else 'FAIL'})")
    ok = ok and orth < 1e-9 and abs(det - 1) < 1e-9

    # 3) 예시 출력(크기 감각). 실제 base 정답은 관측자세 get_coords 실측으로 확정.
    print(f"[3] 예시: camera{p_cam.tolist()} , arm{arm}")
    print(f"        → base = [{p_base[0]:.1f}, {p_base[1]:.1f}, {p_base[2]:.1f}] mm")
    print(f"    (건수님 실사용 로그 base=[448.9,-39.4,26.0] 은 '그때의 관측자세'에서 나온 값 —")
    print(f"     숫자 일치는 그 자세의 get_coords 를 넣어야 재현됨. 여기선 임의 자세라 참고만.)")

    tip = gripper_tip_offset_base(arm)
    print(f"[4] 그리퍼 손끝 오프셋(base) = [{tip[0]:.1f}, {tip[1]:.1f}, {tip[2]:.1f}] mm")

    print("\n" + ("PASS — 변환 수학 자기검증 통과" if ok else "FAIL"))
    return ok


if __name__ == "__main__":
    import sys
    sys.exit(0 if _selftest() else 1)
