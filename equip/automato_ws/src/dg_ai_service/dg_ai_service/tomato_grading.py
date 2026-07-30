"""RP-127  YOLO 검출 -> 등급(Tomato.grade) + 3D 좌표 순수 로직.

rclpy/automato_interfaces 의존이 없어 automato_ws 전용 venv(모델 테스트용)에서도
바로 import/테스트 가능하다(depth_lookup.py와 같은 이유). ROS2 서비스
Request/Response 메시지 조립은 detect_tomatoes_server.py가 담당한다.
"""
from typing import Any, List, Tuple

from dg_ai_service.depth_lookup import deproject_pixel, find_valid_depth
from dg_ai_service.yolo_detector import GRADE_BY_LABEL

DEFAULT_MASK_PADDING_PX = 6.0

# depth 조회 실패 시 중심 픽셀 주변에서 유효 depth를 찾을 최대 반경(픽셀)
DEPTH_SEARCH_RADIUS_PX = 5


def build_tomatoes(
    detector: Any,
    color_img: Any,
    depth_img: Any,
    intrinsics: Tuple[float, float, float, float],
    padding_px: float = DEFAULT_MASK_PADDING_PX,
) -> List[dict]:
    """color/depth 프레임 한 쌍에서 등급·3D 좌표가 채워진 토마토 목록을 만든다.

    GRADE_BY_LABEL에 없는 라벨(unripe 등)과, 주변 탐색으로도 depth를 못 찾은
    검출은 결과에서 제외한다 — Tomato.msg엔 유효성 플래그가 없으므로 신뢰
    못 하는 좌표를 내보내지 않는다.
    """
    fx, fy, ppx, ppy = intrinsics
    tomatoes: List[dict] = []
    for det in detector.detect_instances(color_img, padding_px=padding_px):
        grade = GRADE_BY_LABEL.get(det['label'])
        if grade is None:
            continue
        u, v, depth_mm = find_valid_depth(
            depth_img, det['center_u'], det['center_v'], DEPTH_SEARCH_RADIUS_PX
        )
        if depth_mm <= 0.0:
            continue
        x, y, z = deproject_pixel(u, v, depth_mm / 1000.0, fx, fy, ppx, ppy)
        tomatoes.append({'tomato_id': len(tomatoes), 'grade': grade, 'x': x, 'y': y, 'z': z})
    return tomatoes
