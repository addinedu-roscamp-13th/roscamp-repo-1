#!/usr/bin/env python3
"""RP-110  depth_lookup.find_valid_depth() / deproject_pixel() 단위 테스트.

실행:
  PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pytest src/dg_ai_service/test/test_depth_lookup.py -v
"""
import numpy as np

from dg_ai_service.depth_lookup import deproject_pixel, find_valid_depth


def test_find_valid_depth_center_already_valid():
    depth = np.full((10, 10), 500.0)

    u, v, depth_mm = find_valid_depth(depth, 5, 5, max_radius=5)

    assert (u, v, depth_mm) == (5, 5, 500.0)


def test_find_valid_depth_searches_neighbors_when_center_invalid():
    depth = np.zeros((10, 10))
    depth[5, 7] = 300.0  # row=v=5, col=u=7 (중심에서 우측으로 2px)

    u, v, depth_mm = find_valid_depth(depth, cu=5, cv_=5, max_radius=5)

    assert (u, v, depth_mm) == (7, 5, 300.0)


def test_find_valid_depth_returns_zero_when_nothing_found():
    depth = np.zeros((20, 20))

    u, v, depth_mm = find_valid_depth(depth, cu=10, cv_=10, max_radius=2)

    assert depth_mm == 0.0


def test_find_valid_depth_clamps_out_of_bounds_center():
    depth = np.full((10, 10), 400.0)

    u, v, depth_mm = find_valid_depth(depth, cu=-5, cv_=99, max_radius=1)

    assert (u, v) == (0, 9)
    assert depth_mm == 400.0


def test_deproject_pixel_matches_pinhole_model():
    x, y, z = deproject_pixel(u=110, v=100, depth_m=2.0, fx=100.0, fy=100.0, ppx=100.0, ppy=100.0)

    assert x == 0.2  # (110-100)*2.0/100.0
    assert y == 0.0
    assert z == 2.0
