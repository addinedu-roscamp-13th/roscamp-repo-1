"""depth 이미지에서 픽셀 좌표의 3D 좌표를 구하는 순수 함수 모음 (RP-110).

rclpy/cv_bridge 의존이 없어서 automato_ws 전용 venv(모델 테스트용)에서도
바로 import/테스트 가능하다.
"""
from typing import Tuple


def find_valid_depth(depth_image, cu: int, cv_: int, max_radius: int) -> Tuple[int, int, float]:
    """(cu, cv_) 픽셀의 depth(mm)가 0(무효)이면 주변을 나선형으로 탐색.

    반사/모서리 등으로 bbox 중심 픽셀의 depth가 종종 무효인 경우가 있어
    (mycobot 프로젝트에서 실측 확인), 유효한 값을 찾을 때까지 반경을
    넓혀가며 찾는다. 반환: (사용된 u, v, depth_mm). 못 찾으면 depth_mm=0.0.
    """
    height, width = depth_image.shape[:2]
    cu = min(max(cu, 0), width - 1)
    cv_ = min(max(cv_, 0), height - 1)
    depth_mm = float(depth_image[cv_, cu])
    if depth_mm > 0.0:
        return cu, cv_, depth_mm
    for radius in range(1, max_radius + 1):
        for du in range(-radius, radius + 1):
            for dv in range(-radius, radius + 1):
                if max(abs(du), abs(dv)) != radius:
                    continue  # 이미 검사한 안쪽 반경은 건너뜀
                u, v = cu + du, cv_ + dv
                if not (0 <= u < width and 0 <= v < height):
                    continue
                depth_mm = float(depth_image[v, u])
                if depth_mm > 0.0:
                    return u, v, depth_mm
    return cu, cv_, 0.0


def deproject_pixel(u: int, v: int, depth_m: float, fx: float, fy: float, ppx: float, ppy: float):
    """핀홀 카메라 모델로 픽셀+depth -> 카메라 프레임 3D 좌표(m)."""
    x = (u - ppx) * depth_m / fx
    y = (v - ppy) * depth_m / fy
    z = depth_m
    return x, y, z
