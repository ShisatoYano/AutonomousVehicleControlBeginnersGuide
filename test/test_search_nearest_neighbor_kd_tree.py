"""
Unit test of searching nearest neighbor point simulation by k-d tree

Author: Shisato Yano
"""

import matplotlib as mpl
mpl.use("Agg")
import numpy as np
from pathlib import Path
import sys

sys.path.append(str(Path(__file__).absolute().parent) + "/../src/simulations/perception/point_cloud_search")
sys.path.append(str(Path(__file__).absolute().parent) + "/../src/components/search/kd_tree")
sys.path.append(str(Path(__file__).absolute().parent) + "/../src/components/sensors/lidar")
import search_nearest_neighbor_kd_tree
from kd_tree import KdTree
from scan_point import ScanPoint


def make_scan_point(x_m, y_m):
    distance_m = float(np.hypot(x_m, y_m))
    angle_rad = float(np.arctan2(y_m, x_m))
    return ScanPoint(distance_m, angle_rad, x_m, y_m)


def point_tuple(scan_point):
    point_array = scan_point.get_point_array()
    return (float(point_array[0, 0]), float(point_array[1, 0]))


def test_simulation():
    search_nearest_neighbor_kd_tree.show_plot = False

    search_nearest_neighbor_kd_tree.main()


def test_nearest_neighbor_matches_brute_force():
    scan_points = [
        make_scan_point(-2.0, 0.0),
        make_scan_point(0.0, 0.0),
        make_scan_point(1.5, 0.5),
        make_scan_point(3.0, 0.0),
        make_scan_point(2.0, 2.0),
    ]
    target_point = make_scan_point(1.2, 0.4)
    target_array = target_point.get_point_array()

    expected_point = min(
        scan_points,
        key=lambda scan_point: np.linalg.norm(
            target_array - scan_point.get_point_array()
        ),
    )

    nearest_point = KdTree(scan_points.copy()).search_nearest_neighbor_point(target_point)

    assert point_tuple(nearest_point) == point_tuple(expected_point)


def test_radius_search_returns_only_points_within_radius():
    scan_points = [
        make_scan_point(-1.0, 0.0),
        make_scan_point(0.0, 0.0),
        make_scan_point(0.5, 0.5),
        make_scan_point(1.2, 0.0),
        make_scan_point(2.0, 2.0),
    ]
    target_point = make_scan_point(0.0, 0.0)

    neighbor_points = KdTree(scan_points.copy()).search_neighbor_points_within_r(
        target_point, r=1.0
    )

    assert {point_tuple(point) for point in neighbor_points} == {
        (-1.0, 0.0),
        (0.0, 0.0),
        (0.5, 0.5),
    }
