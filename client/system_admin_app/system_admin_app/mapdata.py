"""SLAM 맵(.pgm + .yaml) 로더.

정적 파일 방식으로 점유격자 맵을 읽어 배경으로 쓸 수 있게 (이미지 배열 + 월드 배치
정보)를 돌려준다. .pgm/.yaml 짝이 함께 있어야 한다(resolution·origin 필요).
"""
from __future__ import annotations

import os
from dataclasses import dataclass

import numpy as np
import yaml


@dataclass
class MapData:
    image: np.ndarray      # (H, W) uint8, row 0 = 이미지 상단
    resolution: float      # m / pixel
    origin_x: float        # 맵 좌하단의 월드 x
    origin_y: float        # 맵 좌하단의 월드 y

    @property
    def height(self) -> int:
        return self.image.shape[0]

    @property
    def width(self) -> int:
        return self.image.shape[1]

    @property
    def width_m(self) -> float:
        return self.width * self.resolution

    @property
    def height_m(self) -> float:
        return self.height * self.resolution


def _read_pgm_p5(path: str) -> np.ndarray:
    """P5(바이너리) PGM을 (H, W) uint8 배열로. 주석(#)과 임의 공백을 처리한다."""
    with open(path, "rb") as f:
        data = f.read()

    # 헤더 토큰 파싱: 매직 → width → height → maxval (주석/공백 건너뜀)
    tokens = []
    i = 0
    n = len(data)
    while len(tokens) < 4 and i < n:
        # 공백 skip
        while i < n and data[i:i + 1].isspace():
            i += 1
        # 주석 skip
        if i < n and data[i:i + 1] == b"#":
            while i < n and data[i:i + 1] not in (b"\n", b"\r"):
                i += 1
            continue
        # 토큰 읽기
        start = i
        while i < n and not data[i:i + 1].isspace():
            i += 1
        tokens.append(data[start:i])

    magic, width, height = tokens[0], int(tokens[1]), int(tokens[2])
    if magic != b"P5":
        raise ValueError(f"P5 PGM만 지원합니다 (got {magic!r})")
    # maxval 다음의 단일 공백 1바이트 뒤부터 픽셀 데이터
    i += 1
    pixels = np.frombuffer(data, dtype=np.uint8, count=width * height, offset=i)
    return pixels.reshape((height, width))


def load_map(yaml_path: str) -> MapData:
    """map.yaml 경로를 받아 MapData 반환. image 경로는 yaml 기준 상대경로 허용."""
    with open(yaml_path, "r") as f:
        meta = yaml.safe_load(f)

    img_name = meta["image"]
    img_path = img_name
    if not os.path.isabs(img_path):
        img_path = os.path.join(os.path.dirname(yaml_path), img_name)

    image = _read_pgm_p5(img_path)
    if int(meta.get("negate", 0)) == 1:
        image = 255 - image

    origin = meta.get("origin", [0.0, 0.0, 0.0])
    return MapData(
        image=image,
        resolution=float(meta["resolution"]),
        origin_x=float(origin[0]),
        origin_y=float(origin[1]),
    )
