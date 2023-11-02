"""
Unit test of Gnss

Author: Shisato Yano
"""

import matplotlib as mpl
mpl.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pytest
import sys
from pathlib import Path

sys.path.append(str(Path(__file__).absolute().parent) + "/../src/components/sensors/gnss")
from gnss import Gnss


# mock class
class MockState:
    def __init__(self):
        pass

    def get_x_m(self):
        return 1.0
    
    def get_y_m(self):
        return 1.0
    
    def get_yaw_rad(self):
        return 1.0
    
    def get_speed_mps(self):
        return 1.0


# test instance
gnss = Gnss()
state = MockState()


def test_initialize():
    assert gnss.NOISE_VAR_MAT.shape == (2, 2)
    assert gnss.NOISE_VAR_MAT[0, 0] == 0.25
    assert gnss.NOISE_VAR_MAT[0, 1] == 0.0
    assert gnss.NOISE_VAR_MAT[1, 0] == 0.0
    assert gnss.NOISE_VAR_MAT[1, 1] == 0.25
    assert gnss.DRAW_COLOR == 'g'
    assert gnss.latest_observed_xy == None
    assert len(gnss.x_history) == 0
    assert len(gnss.y_history) == 0


def test_update():
    gnss.update(state)
    assert gnss.latest_observed_xy.shape == (2, 1)
    assert len(gnss.x_history) != 0
    assert len(gnss.y_history) != 0


def test_get_xy_pos():
    assert gnss.get_xy_pos().shape == (2, 1)


def test_draw():
    plt.clf()
    plt.close()
    
    figure = plt.figure(figsize=(8, 8))
    axes = figure.add_subplot(111)

    gnss.draw(axes, [])
