"""
Unit test of KdTree

Author: Adam Amzar
"""

import numpy as np
import pytest
import sys
from pathlib import Path

sys.path.append(str(Path(__file__).absolute().parent) + "/../src/components/search/kd_tree")
sys.path.append(str(Path(__file__).absolute().parent) + "/../src/components/sensors/lidar")
from kd_tree import KdTree
from scan_point import ScanPoint


def make_points(xy_array):
    return [ScanPoint(0, 0, xy[0], xy[1]) for xy in xy_array]


def distance(point_a, point_b):
    return np.linalg.norm(point_a.get_point_array() - point_b.get_point_array())


@pytest.mark.parametrize("seed, points_num", [(0, 10), (1, 100), (2, 1000)])
def test_nearest_neighbor_matches_brute_force(seed, points_num):
    rng = np.random.default_rng(seed)
    source_points = make_points(rng.random((points_num, 2)))
    kd_tree = KdTree(list(source_points))

    for target in make_points(rng.random((50, 2))):
        found = kd_tree.search_nearest_neighbor_point(target)
        expected_dist = min(distance(target, point) for point in source_points)
        assert distance(target, found) == pytest.approx(expected_dist)


@pytest.mark.parametrize("seed, points_num, r", [(0, 10, 0.3), (1, 100, 0.2), (2, 1000, 0.05)])
def test_neighbor_points_within_r_match_brute_force(seed, points_num, r):
    rng = np.random.default_rng(seed)
    source_points = make_points(rng.random((points_num, 2)))
    kd_tree = KdTree(list(source_points))

    for target in make_points(rng.random((50, 2))):
        found = kd_tree.search_neighbor_points_within_r(target, r=r)
        expected = [point for point in source_points if distance(target, point) <= r]
        assert sorted(map(id, found)) == sorted(map(id, expected))


def test_search_skips_subtrees(monkeypatch):
    rng = np.random.default_rng(3)
    points_num = 1000
    kd_tree = KdTree(make_points(rng.random((points_num, 2))))

    visited = []
    original = KdTree._search_nearest_neighbor_candidates

    def counting(self, target_point_array, search_radius, node):
        if node: visited.append(node)
        original(self, target_point_array, search_radius, node)

    monkeypatch.setattr(KdTree, "_search_nearest_neighbor_candidates", counting)

    for target in make_points(rng.random((20, 2))):
        visited.clear()
        kd_tree.search_nearest_neighbor_point(target)
        assert len(visited) < points_num / 10
