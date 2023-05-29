"""
Unit test of Obstacle

Author: Shisato Yano
"""

import matplotlib as mpl
mpl.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pytest
import sys
from pathlib import Path

sys.path.append(str(Path(__file__).absolute().parent) + "/../src/components/obstacle")
from obstacle import Obstacle


class MockState:
    def __init__(self):
        pass

    def update(self, accel_mps2, yaw_rate_rps, time_s):
        pass
    
    def x_y_yaw(self):
        return np.array([[0.0], [0.0], [0.0]])
    
    def get_x_m(self):
        return 0.0

    def get_y_m(self):
        return 0.0
    
    def get_yaw_rad(self):
        return 0.0
    
    def get_speed_kmph(self):
        return 0.0
    
    def draw(self, axes, elems):
        pass


def test_initialize():
    state = MockState()
    obst = Obstacle(state)
    
    assert obst.state != None
    assert obst.accel_mps2 == 0.0
    assert obst.yaw_rate_rps == 0.0
    assert obst.array != None
    assert obst.array.get_data().shape[0] == 2
    assert obst.array.get_data().shape[1] == 5


def test_set_arguments():
    state = MockState()
    obst = Obstacle(state, accel_mps2=1.0, yaw_rate_rps=2.0, length_m=3.0, width_m=4.0)

    assert obst.accel_mps2 == 1.0
    assert obst.yaw_rate_rps == 2.0
    assert obst.array.get_x_data()[0] == 3.0
    assert obst.array.get_y_data()[0] == 4.0
    assert obst.array.get_x_data()[1] == -3.0
    assert obst.array.get_y_data()[1] == 4.0
    assert obst.array.get_x_data()[2] == -3.0
    assert obst.array.get_y_data()[2] == -4.0
    assert obst.array.get_x_data()[3] == 3.0
    assert obst.array.get_y_data()[3] == -4.0
    assert obst.array.get_x_data()[4] == 3.0
    assert obst.array.get_y_data()[4] == 4.0


def test_update():
    state = MockState()
    obst = Obstacle(state)
    obst.update(1.0)


def test_draw():
    state = MockState()
    obst = Obstacle(state)

    plt.clf()
    plt.close()

    figure = plt.figure(figsize=(8, 8))
    axes = figure.add_subplot(111)

    obst.draw(axes, [])
