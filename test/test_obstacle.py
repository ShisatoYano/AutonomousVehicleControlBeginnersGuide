"""
Unit test of Obstacle

Author: yalshish
"""

import numpy as np
import sys
from pathlib import Path

sys.path.append(str(Path(__file__).absolute().parent) + "/../src/components/obstacle")
sys.path.append(str(Path(__file__).absolute().parent) + "/../src/components/state")
from obstacle import Obstacle
from state import State


LENGTH_M = 2.0
WIDTH_M = 1.0

ROTATED_90DEG_X_M = [-WIDTH_M, -WIDTH_M, WIDTH_M, WIDTH_M, -WIDTH_M]
ROTATED_90DEG_Y_M = [LENGTH_M, -LENGTH_M, -LENGTH_M, LENGTH_M, LENGTH_M]


def test_vertex_xy_at_origin() -> None:
    obstacle = Obstacle(State(0.0, 0.0, 0.0), length_m=LENGTH_M, width_m=WIDTH_M)
    x_m, y_m = obstacle.vertex_xy()

    assert np.allclose(x_m, [LENGTH_M, -LENGTH_M, -LENGTH_M, LENGTH_M, LENGTH_M])
    assert np.allclose(y_m, [WIDTH_M, WIDTH_M, -WIDTH_M, -WIDTH_M, WIDTH_M])


def test_vertex_xy_translated() -> None:
    x_m, y_m = 10.0, -5.0
    obstacle = Obstacle(State(x_m, y_m, 0.0), length_m=LENGTH_M, width_m=WIDTH_M)
    vertex_x_m, vertex_y_m = obstacle.vertex_xy()

    assert np.allclose(vertex_x_m, x_m + np.array([LENGTH_M, -LENGTH_M, -LENGTH_M, LENGTH_M, LENGTH_M]))
    assert np.allclose(vertex_y_m, y_m + np.array([WIDTH_M, WIDTH_M, -WIDTH_M, -WIDTH_M, WIDTH_M]))


def test_vertex_xy_rotated_90deg() -> None:
    obstacle = Obstacle(State(0.0, 0.0, np.deg2rad(90)), length_m=LENGTH_M, width_m=WIDTH_M)
    x_m, y_m = obstacle.vertex_xy()

    assert np.allclose(x_m, ROTATED_90DEG_X_M)
    assert np.allclose(y_m, ROTATED_90DEG_Y_M)


def test_vertex_xy_after_update() -> None:
    obstacle = Obstacle(State(0.0, 0.0, 0.0), yaw_rate_rps=np.pi / 2, length_m=LENGTH_M, width_m=WIDTH_M)
    obstacle.update(1.0)
    x_m, y_m = obstacle.vertex_xy()

    assert np.allclose(x_m, ROTATED_90DEG_X_M)
    assert np.allclose(y_m, ROTATED_90DEG_Y_M)
