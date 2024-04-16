"""
Unit test of ScanPoint

Author: Shisato Yano
"""

import pytest
import sys
from pathlib import Path

sys.path.append(str(Path(__file__).absolute().parent) + "/../src/components/mapping/grid")
from grid_map import GridMap


class MockFloatGrid:
    def __init__(self, value=0.0):
        self.data = value
    
    def get_data(self):
        return self.data


map = GridMap()


def test_initialization():
    assert map.width_m == 60.0
    assert map.height_m == 60.0
    assert map.resolution_m == 5.0
    assert map.center_x_m == 0.0
    assert map.center_y_m == 0.0

    assert map.width_grids_num == 12
    assert map.height_grids_num == 12
    assert map.all_grids_num == 144

    assert map.left_bottom_x_m == -30.0
    assert map.left_bottom_y_m == -30.0

    assert len(map.data) == map.all_grids_num
    assert map.data[0].get_data() == 0.0
    assert map.data[-1].get_data() == 0.0


def test_calculate_xy_index_from_position():
    assert map.calculate_xy_index_from_position(-30.0, map.left_bottom_x_m, map.width_grids_num) == 0
    assert map.calculate_xy_index_from_position(30.0, map.left_bottom_x_m, map.width_grids_num) == 12
    assert map.calculate_xy_index_from_position(-60.0, map.left_bottom_x_m, map.width_grids_num) == None
    assert map.calculate_xy_index_from_position(60.0, map.left_bottom_x_m, map.width_grids_num) == None


def test_calculate_vector_index_from_xy_index():
    assert map.calculate_vector_index_from_xy_index(0, 0) == 0
    assert map.calculate_vector_index_from_xy_index(11, 0) == 11
    assert map.calculate_vector_index_from_xy_index(0, 1) == 12


def test_calculate_vector_index_from_position():
    assert map.calculate_vector_index_from_position(-30, -30) == 0
    assert map.calculate_vector_index_from_position(30, -30) == 12


def test_calculate_xy_index_from_vector_index():
    assert map.calculate_xy_index_from_vector_index(0) == (0, 0)
    assert map.calculate_xy_index_from_vector_index(11) == (11, 0)
    assert map.calculate_xy_index_from_vector_index(12) == (0, 1)


def test_calculate_grid_center_pos_from_index():
    assert map.calculate_grid_center_pos_from_index(0, map.left_bottom_x_m) == -27.5
    assert map.calculate_grid_center_pos_from_index(11, map.left_bottom_x_m) == 27.5


def test_calculate_grid_center_xy_pos_from_xy_index():
    assert map.calculate_grid_center_xy_pos_from_xy_index(0, 0) == (-27.5, -27.5)
    assert map.calculate_grid_center_xy_pos_from_xy_index(11, 11) == (27.5, 27.5)


def test_calculate_grid_center_xy_pos_from_vector_index():
    assert map.calculate_grid_center_xy_pos_from_vector_index(0) == (-27.5, -27.5)
    assert map.calculate_grid_center_xy_pos_from_vector_index(11) == (27.5, -27.5)


def test_set_grid_data():
    map.set_grid_data(0, MockFloatGrid(1.0))
    assert map.data[0].get_data() == 1.0


def test_get_grid_data():
    assert map.get_grid_data(0).get_data() == 1.0
