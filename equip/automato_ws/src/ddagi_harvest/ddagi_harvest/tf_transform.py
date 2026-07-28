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

# 관측자세에서의 get_coords (base←joint6). observe_setup.py 로 확정, 2026-07-24.
# pick.OBSERVE_ANGLES 와 반드시 짝(같은 자세). 관측자세를 바꾸면 이 값도 다시 딴다.
# 검출은 항상 이 고정 자세에서 하므로, camera→base 는 이 한 값으로 계산된다.
OBSERVE_COORDS = [-144.1, -51.9, 253.3, -93.7, -3.0, -87.3]


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


# --------------------------------------------------------------------------- #
# URDF 순기구학(FK)으로 base→joint6 구하기.
# get_coords + 오일러 방식은 pymycobot 프레임과 URDF joint6 프레임이 달라 크게
# 어긋난다(실측 확인: z 211mm 오차). 건수님이 ROS TF(URDF FK)를 쓴 이유. 여기선
# mycobot_280_pi.urdf 의 base→joint6 사슬(j1..j5)을 직접 계산해 같은 프레임을 쓴다.
# 검증: FK 방식 z 오차 3mm (오일러 방식 211mm 대비).
# --------------------------------------------------------------------------- #

# pick.OBSERVE_ANGLES 와 반드시 짝(같은 관측자세). 관측자세 바꾸면 같이 갱신.
OBSERVE_ANGLES = [-3.2, 87.2, -26.4, -64.5, 5.6, -3.4]

# 실측 TF 보정 (관측자세 고정). 이 팔의 관절 0점(엔코더 영점) 미세 오차 때문에
# 명령각(OBSERVE_ANGLES)과 실제 자세가 조금 달라, URDF FK 결과가 계통적으로 어긋난다.
# 검출은 항상 이 고정 관측자세에서만 하므로, 그 자세에서 실측한 상수 보정벡터를
# camera→base 결과에 더해 흡수한다. (tf_verify 게이지 측정 3점 평균, 2026-07-25)
#   측정 3점(camera x=-28.6/+16.9/+139) 오차 y=+50/+60/+65, x=+5/-5/-20, z=+15/+15/+10
#   → 평균 보정 [-6.7, 58.3, 13.3]. 잔차 ~±1.3cm(회전 성분) — 파지 허용범위.
#   근본해결은 팔 J1~J6 0점 재캘리(하지만 모든 티칭좌표 무효화 → 마감 후로).
# 2026-07-28 재측정 — 새 관측자세 + 실측각 FK 기준, 게이지 조그 1점.
#   측정: 실제-계산 = [-15.0, 60.0, 25.0]
# 주목: y≈+60 은 종전 관측자세(+58.3)와 거의 같다. 자세를 크게 바꿨는데(J2 55→87,
# J3 1.6→-26) y 오차가 그대로란 건, 이 성분이 '자세 드리프트'가 아니라 핸드아이/TCP
# 사슬의 고정 오차일 가능성을 시사한다(추후 조사 대상). x·z 는 자세에 따라 변했다.
# ⚠ 1점 측정이라 위치 의존 성분은 미분리 — 좌/우 끝에서 잔차가 남을 수 있다.
TF_OBSERVE_CORRECTION_MM = np.array([-15.0, 60.0, 25.0])

# mycobot_280_pi.urdf 의 base→joint6 관절 사슬 (j1..j5). (xyz[m], rpy[rad]) 고정변환
# + z축 관절회전. 자세한 값은 calib/2_TF핸드아이/handeye_ws/urdf/ 참조.
_URDF_J6_CHAIN = [
    ([0, 0, 0.13956], [0, 0, 0]),
    ([0, 0, -0.001], [0, 1.5708, -1.5708]),
    ([-0.1104, 0, 0], [0, 0, 0]),
    ([-0.096, 0, 0.06462], [0, 0, -1.5708]),
    ([0, -0.07318, -0.001], [1.5708, -1.5708, 0]),
]


def _R_rad(axis: str, rad: float) -> np.ndarray:
    c, s = np.cos(rad), np.sin(rad)
    if axis == "X":
        return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
    if axis == "Y":
        return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def fk_base_to_joint6(angles_deg) -> np.ndarray:
    """URDF 순기구학으로 T(base←joint6) (mm). angles_deg = [j1..j5(..)]."""
    M = np.eye(4)
    for (xyz, rpy), a in zip(_URDF_J6_CHAIN, angles_deg[:5]):
        R = (_R_rad("Z", rpy[2]) @ _R_rad("Y", rpy[1]) @ _R_rad("X", rpy[0])
             @ _R_rad("Z", np.radians(a)))
        Tj = np.eye(4)
        Tj[:3, :3] = R
        Tj[:3, 3] = np.array(xyz) * 1000.0
        M = M @ Tj
    return M


def camera_to_base_fk(p_camera_mm, observe_angles=None) -> np.ndarray:
    """camera(optical) → base, base←joint6를 URDF FK로 (주 경로)."""
    ang = observe_angles if observe_angles is not None else OBSERVE_ANGLES
    p_cam = np.array([*p_camera_mm, 1.0])
    return (fk_base_to_joint6(ang) @ handeye_T() @ p_cam)[:3]


def camera_to_base_at_observe(p_camera_mm, observe_angles=None) -> np.ndarray:
    """검출은 고정 관측자세에서 하므로 관측자세 관절각으로 바로 변환하는 편의 함수.

    AI가 준 토마토 camera 좌표 → base 좌표. 실전 파이프라인의 주 경로 (FK 기반).
    관측자세 실측 상수 보정(TF_OBSERVE_CORRECTION_MM)을 더해 반환한다.
    """
    return camera_to_base_fk(p_camera_mm, observe_angles) + TF_OBSERVE_CORRECTION_MM


# --------------------------------------------------------------------------- #
# 관측자세 camera → flange 직접 변환 (터치 실측 피팅).
#
# 검출은 항상 고정 관측자세에서만 하므로 FK·핸드아이의 곱은 하나의 고정 변환이다.
# 그 변환을 캘리 값으로 조립하는 대신, '손끝이 열매에 닿는 flange 좌표'를 직접 측정해
# 강체(회전+이동)로 피팅했다. 이 방식이 흡수하는 것:
#   · 핸드아이 캘리 오차 / 카메라 마운트 미세 이동
#   · URDF FK 와 pymycobot send_coords 프레임 차(실측 ~8mm)
#   · 손끝-플랜지(TCP) 추정 오차, 서보 처짐의 계통 성분
# 측정이 '우리가 실제로 명령하는 프레임'에 있으므로 결과를 그대로 send_coords 하면 된다.
#
# fit_observe_tf.py, 6점 실측(2026-07-28):
#   자체 잔차 평균 5.9mm / 최대 7.9mm,  LOO 교차검증 평균 9.2mm / 최대 13.4mm
#   affine(12DOF)은 자체 3.3mm 였지만 LOO 14.9mm 로 악화 + 특이값 0.55 붕괴 → 과적합,
#   그래서 rigid 채택. 남는 ~9mm 는 팔 반복정확도(저가 팔의 물리 한계).
# ⚠ 관측자세를 바꾸거나 카메라를 건드리면 이 변환은 무효 → 재측정.
OBSERVE_CAM2FLANGE_R = np.array([
    [-0.046961, +0.036223, +0.998240],
    [-0.998348, -0.034809, -0.045703],
    [+0.033092, -0.998737, +0.037797],
])
OBSERVE_CAM2FLANGE_T = np.array([-272.13, +24.09, +280.12])


def observe_cam_to_flange(p_camera_mm) -> np.ndarray:
    """관측자세 camera(optical) 좌표 → 손끝이 그 열매에 닿는 flange 좌표(mm).

    이 값이 곧 send_coords 목표다(TCP·처짐·DESCEND 보정 불필요 — 측정에 포함됨).
    """
    return OBSERVE_CAM2FLANGE_R @ np.asarray(p_camera_mm, float) + OBSERVE_CAM2FLANGE_T


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
