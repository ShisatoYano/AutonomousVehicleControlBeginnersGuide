"""
Unit test of AStarPathPlanner and DijkstraPathPlanner on small grids

Author: abedinm
"""

import json
import sys
from pathlib import Path

import numpy as np
import numpy.typing as npt
import pytest

abs_dir_path = str(Path(__file__).absolute().parent)
sys.path.append(abs_dir_path + "/../src/components/plan/astar")
sys.path.append(abs_dir_path + "/../src/components/plan/dijkstra")
sys.path.append(abs_dir_path + "/../src/components/visualization")
from astar_path_planner import AStarPathPlanner
from dijkstra_path_planner import DijkstraPathPlanner
from min_max import MinMax

# The planners run the search in their constructor, so each test writes a small
# grid to pytest's tmp_path and reads the saved path back. No plot is drawn.
# Every cell is 1 m wide and the map starts at 0 m, so grid indices and world
# coordinates are the same numbers.
PLANNERS = [AStarPathPlanner, DijkstraPathPlanner]


def plan(planner_class: type, grid: npt.NDArray[np.float64], start: tuple[int, int],
         goal: tuple[int, int], tmp_path: Path) -> list[tuple[int, int]]:
    """Run the planner on the grid and return the saved path as a list of (x, y)."""
    map_file = tmp_path / "grid.npy"
    path_file = tmp_path / "path.json"
    np.save(map_file, grid)

    rows, cols = grid.shape
    planner_class(start, goal, str(map_file),
                  x_lim=MinMax(0, cols), y_lim=MinMax(0, rows),
                  path_filename=str(path_file), gif_name=None)

    with open(path_file) as f:
        return [tuple(point) for point in json.load(f)]


@pytest.mark.parametrize("planner_class", PLANNERS)
def test_empty_grid_path_goes_straight_from_start_to_goal(planner_class: type, tmp_path: Path) -> None:
    grid = np.zeros((10, 10))

    path = plan(planner_class, grid, (0, 0), (9, 0), tmp_path)

    assert path[0] == (0, 0)
    assert path[-1] == (9, 0)
    # Along one row the only shortest path visits all 10 cells in order
    assert path == [(x, 0) for x in range(10)]


@pytest.mark.parametrize("planner_class", PLANNERS)
def test_empty_grid_path_steps_to_neighboring_cells_only(planner_class: type, tmp_path: Path) -> None:
    grid = np.zeros((10, 10))

    path = plan(planner_class, grid, (0, 0), (6, 3), tmp_path)

    assert path[0] == (0, 0)
    assert path[-1] == (6, 3)
    for (x1, y1), (x2, y2) in zip(path, path[1:]):
        assert max(abs(x2 - x1), abs(y2 - y1)) == 1


def test_dijkstra_path_on_empty_grid_has_the_shortest_length(tmp_path: Path) -> None:
    grid = np.zeros((10, 10))

    path = plan(DijkstraPathPlanner, grid, (0, 0), (6, 3), tmp_path)

    # 3 diagonal steps and 3 straight steps are the shortest way to (6, 3)
    assert len(path) == 7
    length = sum(np.hypot(x2 - x1, y2 - y1) for (x1, y1), (x2, y2) in zip(path, path[1:]))
    assert np.isclose(length, 3 * np.sqrt(2) + 3)


@pytest.mark.parametrize("planner_class", PLANNERS)
def test_path_goes_through_the_only_gap_in_a_wall(planner_class: type, tmp_path: Path) -> None:
    grid = np.zeros((9, 9))
    grid[:, 4] = 1  # wall at x = 4 for every y ...
    grid[6, 4] = 0  # ... except one gap at y = 6

    path = plan(planner_class, grid, (0, 0), (8, 0), tmp_path)

    assert path[0] == (0, 0)
    assert path[-1] == (8, 0)
    assert len(path) <= 20  # the saved path is the full path, not a thinned one
    cells_in_wall = [(x, y) for x, y in path if x == 4]
    assert cells_in_wall == [(4, 6)]


@pytest.mark.parametrize("planner_class", PLANNERS)
def test_path_never_enters_an_obstacle_cell(planner_class: type, tmp_path: Path) -> None:
    grid = np.zeros((9, 9))
    grid[2:7, 3:6] = 1  # a block between the start and the goal

    path = plan(planner_class, grid, (0, 4), (8, 4), tmp_path)

    assert len(path) <= 20
    for x, y in path:
        assert grid[int(y), int(x)] == 0
