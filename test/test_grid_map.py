"""
Unit test of GridMap

Author: Dipak Chaudhari
"""

import sys
from pathlib import Path

sys.path.append(str(Path(__file__).absolute().parent) + "/../src/components/mapping/grid")
from grid_map import GridMap


# 10[m] x 10[m] map with 1[m] grids, from (-5, -5) to (5, 5)
grid_map = GridMap(width_m=10.0, height_m=10.0, resolution_m=1.0, center_x_m=0.0, center_y_m=0.0)


def test_vector_index_inside_map():
    assert grid_map.calculate_vector_index_from_position(-5.0, -5.0) == 0
    assert grid_map.calculate_vector_index_from_position(0.5, 0.5) == 55
    assert grid_map.calculate_vector_index_from_position(4.9, 4.9) == 99


def test_vector_index_outside_map_is_none():
    assert grid_map.calculate_vector_index_from_position(100.0, 0.0) is None
    assert grid_map.calculate_vector_index_from_position(0.0, -6.0) is None
    assert grid_map.calculate_vector_index_from_position(-100.0, 100.0) is None


def test_vector_index_on_far_edge_is_none():
    # x = 5.0 is the right edge, one grid past the last column
    assert grid_map.calculate_vector_index_from_position(5.0, 0.0) is None
    assert grid_map.calculate_vector_index_from_position(0.0, 5.0) is None
