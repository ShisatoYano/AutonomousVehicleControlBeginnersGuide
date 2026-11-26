"""
Unit test of DwaController

Author: Yuvraj Gupta
"""

import numpy as np
import pytest
import sys
from pathlib import Path

sys.path.append(str(Path(__file__).absolute().parent) + "/../src/components/plan/dwa")
sys.path.append(str(Path(__file__).absolute().parent) + "/../src/components/vehicle")
sys.path.append(str(Path(__file__).absolute().parent) + "/../src/components/state")
sys.path.append(str(Path(__file__).absolute().parent) + "/../src/components/obstacle")
from dwa_controller import DwaController
from vehicle_specification import VehicleSpecification
from state import State
from obstacle import Obstacle
from obstacle_list import ObstacleList


def _controller(goal_xy=(10.0, 0.0), obst_list=None):
    return DwaController(VehicleSpecification(), goal_xy=goal_xy, obst_list=obst_list)


def test_dynamic_window_clips_to_vehicle_limits_at_zero_speed():
    dwa = _controller()

    min_v, max_v, min_w, max_w = dwa._dynamic_window(0.0, 0.0, 0.1)

    assert min_v == pytest.approx(dwa.MIN_SPEED_MPS)  # can't go below 0 m/s
    assert max_v == pytest.approx(dwa.MAX_ACCEL_MPS2 * 0.1)
    assert min_w == pytest.approx(-dwa.MAX_YAW_ACCEL_RPS2 * 0.1)
    assert max_w == pytest.approx(dwa.MAX_YAW_ACCEL_RPS2 * 0.1)


def test_dynamic_window_clips_to_max_speed_and_yaw_rate():
    dwa = _controller()

    _, max_v, _, max_w = dwa._dynamic_window(dwa.MAX_SPEED_MPS, dwa.MAX_YAW_RATE_RPS, 0.1)

    assert max_v == pytest.approx(dwa.MAX_SPEED_MPS)
    assert max_w == pytest.approx(dwa.MAX_YAW_RATE_RPS)


def test_clearance_is_negative_through_an_obstacle():
    obst_list = ObstacleList()
    obst_list.add_obstacle(Obstacle(State(x_m=5.0, y_m=0.0), length_m=1.0, width_m=1.0))
    dwa = _controller(obst_list=obst_list)

    traj = dwa._predict_trajectory(0.0, 0.0, 0.0, 3.0, 0.0)  # straight through the obstacle

    assert dwa._clearance(traj, dwa._obstacle_points()) < 0.0


def test_clearance_without_obstacles_is_max():
    dwa = _controller()

    traj = dwa._predict_trajectory(0.0, 0.0, 0.0, 3.0, 0.0)

    assert dwa._clearance(traj, dwa._obstacle_points()) == pytest.approx(dwa.MAX_CLEARANCE_M)


def test_heading_score_is_one_toward_goal_and_zero_away():
    dwa = _controller(goal_xy=(100.0, 0.0))

    toward = dwa._predict_trajectory(0.0, 0.0, 0.0, 2.0, 0.0)
    away = dwa._predict_trajectory(0.0, 0.0, np.pi, 2.0, 0.0)

    assert dwa._heading_score(toward) == pytest.approx(1.0)
    assert dwa._heading_score(away) == pytest.approx(0.0)
