"""
Unit test of OptimizedLShapeDetector

Author: Leonardo Martins
"""

import numpy as np
import sys
from pathlib import Path

sys.path.append(str(Path(__file__).absolute().parent) + "/../src/components/detection/l_shape_fitting")
sys.path.append(str(Path(__file__).absolute().parent) + "/../src/components/sensors/lidar")
from robust_least_squares_detector import OptimizedLShapeDetector
from scan_point import ScanPoint


CENTER_X_M, CENTER_Y_M = 10.0, 5.0
LENGTH_M, WIDTH_M = 4.0, 2.0
YAW_DEG = 30.0
NOISE_STD_M = 0.05


def _noisy_l_shape_xy() -> list[tuple[float, float]]:
    """
    x-y[m] of points every 0.1 m on two sides of a known rectangle that meet
    at one corner, with fixed Gaussian noise, like the "L" a laser scanner sees on a car
    """

    yaw_rad = np.deg2rad(YAW_DEG)
    length_dir = np.array([np.cos(yaw_rad), np.sin(yaw_rad)])
    width_dir = np.array([-np.sin(yaw_rad), np.cos(yaw_rad)])
    corner = np.array([CENTER_X_M, CENTER_Y_M]) - length_dir * LENGTH_M / 2 - width_dir * WIDTH_M / 2

    points = [corner + length_dir * s for s in np.linspace(0.0, LENGTH_M, 41)]
    points += [corner + width_dir * s for s in np.linspace(0.1, WIDTH_M, 20)]
    noise = np.random.default_rng(0).normal(scale=NOISE_STD_M, size=(len(points), 2))
    return [(float(x), float(y)) for x, y in np.array(points) + noise]


def _point_cloud(xy_list: list[tuple[float, float]]) -> list[ScanPoint]:
    return [ScanPoint(float(np.hypot(x, y)), float(np.arctan2(y, x)), x, y) for x, y in xy_list]


def _fit(xy_list: list[tuple[float, float]]) -> list[list[float]]:
    """
    Run a new detector on a new point cloud of the given points
    Return the edge coefficients [a, b, c] of the one detected rectangle
    """

    detector = OptimizedLShapeDetector(show_logger_debug=False)
    detector.update(_point_cloud(xy_list))
    assert len(detector.latest_rectangles_list) == 1
    rectangle = detector.latest_rectangles_list[0]
    return [list(rectangle.a), list(rectangle.b), list(rectangle.c)]


def test_same_points_give_identical_rectangle():
    xy_list = _noisy_l_shape_xy()

    first = _fit(xy_list)
    for _ in range(5):
        assert _fit(xy_list) == first


def test_shuffled_points_give_identical_rectangle():
    xy_list = _noisy_l_shape_xy()
    first = _fit(xy_list)

    rng = np.random.default_rng(1)
    for _ in range(5):
        shuffled = [xy_list[i] for i in rng.permutation(len(xy_list))]
        assert _fit(shuffled) == first
