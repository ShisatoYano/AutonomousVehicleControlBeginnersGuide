"""
Unit test of State

Author: Shisato Yano
"""

import pytest
import sys
import os
import numpy as np

sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../src/state")
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


def test_update():
    updated_array = state.update(1.0, 0.0, 1.0, 2.0)
    updated_x_y_yaw = updated_array.x_y_yaw()
    assert round(updated_x_y_yaw[0, 0], 1) == 0.0
    assert round(updated_x_y_yaw[1, 0], 1) == 4.0
    assert round(updated_x_y_yaw[2, 0], 1) == round(np.deg2rad(90), 1)
