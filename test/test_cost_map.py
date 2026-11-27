"""
Unit test of CostMap

Author: Samarth Mehta
"""

import sys
from pathlib import Path

sys.path.append(str(Path(__file__).absolute().parent) + "/../src/components/mapping/cost")
from cost_map import CostMap


def create_cost_map():
    # 10[m] x 10[m] map with 1[m] grids, from (-5, -5) to (5, 5)
    return CostMap(width_m=10.0, height_m=10.0, resolution_m=1.0,
                   center_x_m=0.0, center_y_m=0.0,
                   max_cost=5.0, obstacle_cost=3.0, free_cost=0.0)


def test_two_points_in_same_grid_add_cost_twice():
    cost_map = create_cost_map()

    # both points are in the grid from (0, 0) to (1, 1)
    cost_map.update_map([0.2, 0.7], [0.3, 0.8], cost_increment=1.0)

    assert cost_map.get_cost(0.5, 0.5) == 2.0


def test_grids_without_points_keep_zero_cost():
    cost_map = create_cost_map()

    cost_map.update_map([0.2, 0.7], [0.3, 0.8], cost_increment=1.0)

    hit_index = cost_map.map.calculate_vector_index_from_position(0.5, 0.5)
    for index in range(cost_map.map.all_grids_num):
        if index != hit_index:
            assert cost_map.map.get_grid_data(index).get_data() == 0.0


def test_cost_does_not_exceed_max_cost_in_one_update():
    cost_map = create_cost_map()

    # 10 points in the same grid, 10 x 1.0 is over max_cost(5.0)
    cost_map.update_map([0.5] * 10, [0.5] * 10, cost_increment=1.0)

    assert cost_map.get_cost(0.5, 0.5) == 5.0


def test_cost_does_not_exceed_max_cost_over_many_updates():
    cost_map = create_cost_map()

    for _ in range(10):
        cost_map.update_map([0.5], [0.5], cost_increment=1.0)

    assert cost_map.get_cost(0.5, 0.5) == 5.0


def test_set_cost_inside_range_is_kept():
    cost_map = create_cost_map()

    cost_map.set_cost(0.5, 0.5, 3.0)

    assert cost_map.get_cost(0.5, 0.5) == 3.0


def test_set_cost_above_max_cost_is_clamped_to_max_cost():
    cost_map = create_cost_map()

    cost_map.set_cost(0.5, 0.5, 100.0)

    assert cost_map.get_cost(0.5, 0.5) == 5.0


def test_set_cost_below_free_cost_is_clamped_to_free_cost():
    cost_map = create_cost_map()

    cost_map.set_cost(0.5, 0.5, -10.0)

    assert cost_map.get_cost(0.5, 0.5) == 0.0


def test_get_cost_outside_map_is_none():
    cost_map = create_cost_map()

    assert cost_map.get_cost(100.0, 0.0) is None
    assert cost_map.get_cost(0.0, -6.0) is None
    # x = 5.0 is the right edge, one grid past the last column
    assert cost_map.get_cost(5.0, 0.0) is None
