"""
Unit test of the coordinates of the path saved by AStarPathPlanner and DijkstraPathPlanner

The saved path must be in world coordinates [m] whatever its length. The map
here starts at (-5, -5), so grid indices and world coordinates differ.
"""

import json
import sys
from pathlib import Path

import numpy as np
import pytest

abs_dir_path = str(Path(__file__).absolute().parent)
sys.path.append(abs_dir_path + "/../src/components/plan/astar")
sys.path.append(abs_dir_path + "/../src/components/plan/dijkstra")
sys.path.append(abs_dir_path + "/../src/components/visualization")
from astar_path_planner import AStarPathPlanner
from dijkstra_path_planner import DijkstraPathPlanner
from min_max import MinMax

PLANNERS = [AStarPathPlanner, DijkstraPathPlanner]


def plan(planner_class, grid, start, goal, x_lim, y_lim, tmp_path):
    """Run the planner on the grid and return the saved path as a list of (x, y)."""
    map_file = tmp_path / "grid.npy"
    path_file = tmp_path / "path.json"
    np.save(map_file, grid)

    planner_class(start, goal, str(map_file), x_lim=x_lim, y_lim=y_lim,
                  path_filename=str(path_file), gif_name=None)

    with open(path_file) as f:
        return [tuple(point) for point in json.load(f)]


@pytest.mark.parametrize("planner_class", PLANNERS)
def test_short_path_is_saved_in_world_coordinates(planner_class, tmp_path):
    # 10 x 10 cells of 1 m from -5 m to 5 m: cell (0, 0) is at (-5, -5)
    grid = np.zeros((10, 10))

    path = plan(planner_class, grid, (-5, -5), (-1, -5),
                MinMax(-5, 5), MinMax(-5, 5), tmp_path)

    assert path == [(-5, -5), (-4, -5), (-3, -5), (-2, -5), (-1, -5)]


@pytest.mark.parametrize("planner_class", PLANNERS)
def test_long_path_is_saved_in_world_coordinates(planner_class, tmp_path):
    # 30 cells along x, so the path has more than 20 points and is thinned to 20
    grid = np.zeros((3, 30))

    path = plan(planner_class, grid, (-5, -5), (24, -5),
                MinMax(-5, 25), MinMax(-5, -2), tmp_path)

    assert len(path) == 20
    assert path[0] == (-5, -5)
    assert path[-1] == (24, -5)
