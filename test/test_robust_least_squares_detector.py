"""
Unit test of OptimizedLShapeDetector

Author: Leonardo Martins
"""

import numpy as np
import numpy.typing as npt
import sys
from pathlib import Path

sys.path.append(str(Path(__file__).absolute().parent) + "/../src/components/detection/l_shape_fitting")
sys.path.append(str(Path(__file__).absolute().parent) + "/../src/components/sensors/lidar")
from rectangle import Rectangle
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


def test_one_ulp_does_not_move_unseen_edges() -> None:
    # Six returns from one visible face of an obstacle in the simulation.
    points = [
        (-9.43, 24.90), (-8.02, 25.06), (-7.48, 25.40),
        (-6.39, 25.43), (-5.47, 25.64), (-4.62, 25.72),
    ]

    def fit_corners_m(xy_list: list[tuple[float, float]]) -> npt.NDArray[np.float64]:
        """Return the four fitted (x, y) corners as a (4, 2) array in metres."""
        rectangle = Rectangle(*_fit(xy_list))
        return rectangle.contour.get_data()[:, :4].T

    initial_corners_m = fit_corners_m(points)
    max_shift_m = 0.0
    for index, (px, py) in enumerate(points):
        perturbed = points.copy()
        perturbed[index] = (float(np.nextafter(px, np.inf)), py)
        shifts_m = np.linalg.norm(fit_corners_m(perturbed) - initial_corners_m, axis=1)
        max_shift_m = max(max_shift_m, float(np.max(shifts_m)))

    assert max_shift_m < 0.001
