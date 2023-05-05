"""
Unit test of State

Author: Shisato Yano
"""

import pytest
import sys
import numpy as np
from pathlib import Path

sys.path.append(str(Path(__file__).absolute().parent) + "/../src/state")
from state import State


# test instance
state = State(0.0, 1.0, np.deg2rad(90), 3.0)


def test_get_x_m():
    assert state.get_x_m() == 0.0


def test_get_y_m():
    assert state.get_y_m() == 1.0


def test_x_y_yaw():
    x_y_yaw_array = state.x_y_yaw()
    assert x_y_yaw_array[0, 0] == 0.0
    assert x_y_yaw_array[1, 0] == 1.0
    assert x_y_yaw_array[2, 0] == np.deg2rad(90)


def test_get_speed():
    assert round(state.get_speed_mps(), 1) == 3.0
    assert round(state.get_speed_kmph(), 1) == 10.8


def test_update():
    updated_array = state.update(1.0, 0.0, 1.0)
    updated_x_y_yaw = updated_array.x_y_yaw()
    assert round(updated_x_y_yaw[0, 0], 1) == 0.0
    assert round(updated_x_y_yaw[1, 0], 1) == 4.0
    assert round(updated_x_y_yaw[2, 0], 1) == round(np.deg2rad(90), 1)
    assert round(updated_array.get_speed_mps(), 1) == 4.0
    assert round(updated_array.get_speed_kmph(), 1) == 14.4


def test_stop_speed():
    updated_array = state.update(-2.9, 0.0, 1.0)
    assert round(updated_array.get_speed_mps(), 1) == 0.0
    assert round(updated_array.get_speed_kmph(), 1) == 0.0


def test_max_speed():
    updated_array = state.update(14.0, 0.0, 1.0)
    assert round(updated_array.get_speed_mps(), 1) == 16.7
    assert round(updated_array.get_speed_kmph(), 1) == 60.0


def test_min_speed():
    updated_array = state.update(-5.9, 0.0, 1.0)
    assert round(updated_array.get_speed_mps(), 1) == -2.8
    assert round(updated_array.get_speed_kmph(), 1) == -10.0
