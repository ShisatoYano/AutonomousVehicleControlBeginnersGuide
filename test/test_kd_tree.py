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
test_point_cloud.append(MockScanPoint(0.588, 0.897))
test_point_cloud.append(MockScanPoint(0.035, 0.69))
test_point_cloud.append(MockScanPoint(0.89, 0.81))
test_point_cloud.append(MockScanPoint(0.37, 0.51))
test_point_cloud.append(MockScanPoint(0.27, 0.71))
test_point_cloud.append(MockScanPoint(0.23, 0.25))
test_point_cloud.append(MockScanPoint(0.11, 0.75))
test_point_cloud.append(MockScanPoint(0.77, 0.03))
test_point_cloud.append(MockScanPoint(0.78, 0.85))
test_point_cloud.append(MockScanPoint(0.65, 0.19))


def test_search_nearest_neighbor_point():
    # target data
    target_point_1 = MockScanPoint(0.035, 0.51)
    target_point_2 = MockScanPoint(0.05, 0.9)
    target_point_3 = MockScanPoint(0.6, 0.5)
    target_point_4 = MockScanPoint(0.8, 0.9)

    kd_tree = KdTree(test_point_cloud)

    nearest_point_1 = kd_tree.search_nearest_neighbor_point(target_point_1)
    nearest_xy_1 = nearest_point_1.get_point_array()
    assert nearest_xy_1[0] == 0.035
    assert nearest_xy_1[1] == 0.69

    nearest_point_2 = kd_tree.search_nearest_neighbor_point(target_point_2)
    nearest_xy_2 = nearest_point_2.get_point_array()
    assert nearest_xy_2[0] == 0.11
    assert nearest_xy_2[1] == 0.75

    nearest_point_3 = kd_tree.search_nearest_neighbor_point(target_point_3)
    nearest_xy_3 = nearest_point_3.get_point_array()
    assert nearest_xy_3[0] == 0.65
    assert nearest_xy_3[1] == 0.19

    nearest_point_4 = kd_tree.search_nearest_neighbor_point(target_point_4)
    nearest_xy_4 = nearest_point_4.get_point_array()
    assert nearest_xy_4[0] == 0.78
    assert nearest_xy_4[1] == 0.85
