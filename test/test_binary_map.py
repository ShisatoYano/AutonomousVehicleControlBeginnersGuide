"""
Unit test of BinaryMap

Author: Pranav Deshpande
"""

import sys
from pathlib import Path

sys.path.append(str(Path(__file__).absolute().parent) + "/../src/components/mapping/binary")
from binary_map import BinaryMap


def make_map():
    # 10[m] x 10[m] map with 1[m] cells, from (-5, -5) to (5, 5)
    # vector index = y_idx * 10 + x_idx, x_idx = floor(x + 5), y_idx = floor(y + 5)
    return BinaryMap(width_m=10.0, height_m=10.0, resolution_m=1.0, center_x_m=0.0, center_y_m=0.0)


def occupied_indices(binary_map):
    # same threshold as BinaryMap.draw_map()
    return [i for i in range(binary_map.map.all_grids_num)
            if binary_map.map.get_grid_data(i).get_data() > 0.5]


def test_new_map_has_no_occupied_cell():
    assert occupied_indices(make_map()) == []


def test_single_point_occupies_only_its_cell():
    binary_map = make_map()

    # x_idx = floor(2.5 + 5) = 7, y_idx = floor(-1.5 + 5) = 3 -> 3 * 10 + 7 = 37
    binary_map.update_map([2.5], [-1.5])

    assert occupied_indices(binary_map) == [37]


def test_point_on_cell_border_occupies_upper_cell():
    binary_map = make_map()

    # x_idx = floor(0 + 5) = 5, y_idx = floor(0 + 5) = 5 -> 5 * 10 + 5 = 55
    binary_map.update_map([0.0], [0.0])

    assert occupied_indices(binary_map) == [55]


def test_point_in_offset_non_square_map_occupies_expected_cell():
    # 4[m] x 2[m] map with 1[m] cells centered at (10, 20), from (8, 19) to (12, 21)
    # vector index = y_idx * 4 + x_idx
    binary_map = BinaryMap(width_m=4.0, height_m=2.0, resolution_m=1.0, center_x_m=10.0, center_y_m=20.0)

    # x_idx = floor(11.5 - 8) = 3, y_idx = floor(20.5 - 19) = 1 -> 1 * 4 + 3 = 7
    binary_map.update_map([11.5], [20.5])

    assert occupied_indices(binary_map) == [7]


def test_points_outside_map_are_ignored():
    binary_map = make_map()

    # right/top edges (5.0) belong to no cell, the others are beyond the map
    outside_x = [5.0, -5.1, 0.0, 0.0, 100.0]
    outside_y = [0.0, 0.0, 5.0, -5.01, -100.0]
    binary_map.update_map(outside_x, outside_y)

    assert occupied_indices(binary_map) == []


def test_only_points_inside_map_are_occupied_when_mixed_with_outside_points():
    binary_map = make_map()

    # (-5, -5) is the bottom left corner -> cell 0, (4.9, 4.9) -> 9 * 10 + 9 = 99
    binary_map.update_map([-5.0, 6.0, 4.9, -20.0], [-5.0, 0.0, 4.9, 3.0])

    assert occupied_indices(binary_map) == [0, 99]


def test_updating_twice_with_same_point_gives_same_map():
    once_map = make_map()
    once_map.update_map([2.5], [-1.5])

    twice_map = make_map()
    twice_map.update_map([2.5], [-1.5])
    twice_map.update_map([2.5], [-1.5])

    assert occupied_indices(twice_map) == occupied_indices(once_map) == [37]
    once_values = [once_map.map.get_grid_data(i).get_data() for i in range(once_map.map.all_grids_num)]
    twice_values = [twice_map.map.get_grid_data(i).get_data() for i in range(twice_map.map.all_grids_num)]
    assert twice_values == once_values
