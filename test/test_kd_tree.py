"""
Unit test of KdTree

Author: Shisato Yano
"""

import pytest
import sys
import os
import numpy as np
from pathlib import Path

sys.path.append(str(Path(__file__).absolute().parent) + "/../src/components/search/kd_tree")
from kd_tree import KdTree


# mock classes
class MockXYArray:
    def __init__(self, data):
        self.data = data
    
    def get_data(self):
        return self.data
    
    def get_dimension(self):
        return self.data.ndim


class MockScanPoint:
    def __init__(self, x_m, y_m):
        self.point_array = MockXYArray(np.array([[x_m], [y_m]]))
    
    def get_point_array(self):
        return self.point_array.get_data()
    
    def get_dimension(self):
        return self.point_array.get_dimension()


# test data
test_point_cloud = []
test_point_cloud.append(MockScanPoint(6, 1))
test_point_cloud.append(MockScanPoint(5, 5))
test_point_cloud.append(MockScanPoint(7, 4))
test_point_cloud.append(MockScanPoint(2, 3))
test_point_cloud.append(MockScanPoint(3, 7))
test_point_cloud.append(MockScanPoint(9, 3))
test_point_cloud.append(MockScanPoint(9, 6))


def test_search_nearest_neighbor_point():
    # target data
    target_point_1 = MockScanPoint(7, 5)
    target_point_2 = MockScanPoint(5, 4)
    target_point_3 = MockScanPoint(9, 2)
    target_point_4 = MockScanPoint(3, 6)

    kd_tree = KdTree(test_point_cloud)

    nearest_point_1 = kd_tree.search_nearest_neighbor_point(target_point_1)
    nearest_xy_1 = nearest_point_1.get_point_array()
    assert nearest_xy_1[0] == 7
    assert nearest_xy_1[1] == 4

    nearest_point_2 = kd_tree.search_nearest_neighbor_point(target_point_2)
    nearest_xy_2 = nearest_point_2.get_point_array()
    assert nearest_xy_2[0] == 5
    assert nearest_xy_2[1] == 5

    nearest_point_3 = kd_tree.search_nearest_neighbor_point(target_point_3)
    nearest_xy_3 = nearest_point_3.get_point_array()
    assert nearest_xy_3[0] == 9
    assert nearest_xy_3[1] == 3

    nearest_point_4 = kd_tree.search_nearest_neighbor_point(target_point_4)
    nearest_xy_4 = nearest_point_4.get_point_array()
    assert nearest_xy_4[0] == 3
    assert nearest_xy_4[1] == 7


def test_search_neighbor_points_within_r():
    # target data
    target_point = MockScanPoint(7, 5)

    kd_tree = KdTree(test_point_cloud)

    neighbor_points = kd_tree.search_neighbor_points_within_r(target_point, r=2)
    
    assert len(neighbor_points) == 2
    
    neighbor_point_1_xy = neighbor_points[0].get_point_array()
    assert neighbor_point_1_xy[0] == 7
    assert neighbor_point_1_xy[1] == 4

    neighbor_point_2_xy = neighbor_points[1].get_point_array()
    assert neighbor_point_2_xy[0] == 5
    assert neighbor_point_2_xy[1] == 5
