"""
Unit test of LShapeFittingDetector

Author: Tejesh
"""

import numpy as np
import numpy.typing as npt
import sys
from pathlib import Path

sys.path.append(str(Path(__file__).absolute().parent) + "/../src/components/detection/l_shape_fitting")
sys.path.append(str(Path(__file__).absolute().parent) + "/../src/components/sensors/lidar")
from l_shape_fitting_detector import LShapeFittingDetector
from rectangle import Rectangle
from scan_point import ScanPoint


CENTER_X_M, CENTER_Y_M = 10.0, 5.0
LENGTH_M, WIDTH_M = 4.0, 2.0
YAW_DEG = 30.0


def _l_shape_point_cloud() -> list[ScanPoint]:
    """
    Points every 0.2 m on two sides of the known rectangle that meet at one corner,
    like the "L" a laser scanner sees on a car
    """

    yaw_rad = np.deg2rad(YAW_DEG)
    length_dir = np.array([np.cos(yaw_rad), np.sin(yaw_rad)])
    width_dir = np.array([-np.sin(yaw_rad), np.cos(yaw_rad)])
    corner = np.array([CENTER_X_M, CENTER_Y_M]) - length_dir * LENGTH_M / 2 - width_dir * WIDTH_M / 2

    points = [corner + length_dir * s for s in np.linspace(0.0, LENGTH_M, 21)]
    points += [corner + width_dir * s for s in np.linspace(0.2, WIDTH_M, 10)]
    return [ScanPoint(float(np.hypot(x, y)), float(np.arctan2(y, x)), float(x), float(y)) for x, y in points]


def _fit_known_rectangle() -> list[Rectangle]:
    detector = LShapeFittingDetector()
    detector.update(_l_shape_point_cloud())
    return detector.latest_rectangles_list


def _long_and_short_sides(rectangle: Rectangle) -> tuple[npt.NDArray[np.float64], npt.NDArray[np.float64]]:
    """
    Return the long and the short side of the rectangle as [dx, dy] vectors[m],
    taken from its first three contour points
    """

    x = np.array(rectangle.contour_x[:3], dtype=float)
    y = np.array(rectangle.contour_y[:3], dtype=float)
    side_1 = np.array([x[1] - x[0], y[1] - y[0]])
    side_2 = np.array([x[2] - x[1], y[2] - y[1]])
    if np.hypot(*side_1) >= np.hypot(*side_2):
        return side_1, side_2
    return side_2, side_1


def test_l_shape_gives_one_rectangle():
    assert len(_fit_known_rectangle()) == 1


def test_long_side_points_in_known_yaw():
    long_side, _ = _long_and_short_sides(_fit_known_rectangle()[0])

    long_side_yaw_deg = np.rad2deg(np.arctan2(long_side[1], long_side[0])) % 180.0
    assert np.isclose(long_side_yaw_deg, YAW_DEG, atol=0.5)


def test_sides_have_known_length_and_width():
    long_side, short_side = _long_and_short_sides(_fit_known_rectangle()[0])

    assert np.isclose(np.hypot(*long_side), LENGTH_M, atol=0.01)
    assert np.isclose(np.hypot(*short_side), WIDTH_M, atol=0.01)
