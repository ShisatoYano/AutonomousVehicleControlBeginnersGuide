"""
Unit test of SensorParameters

Author: Shisato Yano
"""

import matplotlib as mpl
mpl.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pytest
import sys
from pathlib import Path

sys.path.append(str(Path(__file__).absolute().parent) + "/../src/sensors")
from sensor_parameters import SensorParameters


class MockState:
    def __init__(self):
        pass
    
    def x_y_yaw(self):
        return np.array([[1.0], [1.0], [np.deg2rad(90)]])


def test_initialize():
    param = SensorParameters()

    assert param.INST_LON_M == 0.0
    assert param.INST_LAT_M == 0.0
    assert param.MIN_RANGE_M == 0.5
    assert param.MAX_RANGE_M == 40.0
    assert param.RESO_RAD == np.deg2rad(2.0)
    assert param.ANGLE_STD_SCALE == 0.01
    assert param.DIST_STD_RATE == 0.005
    assert param.inst_pos_array != None
    assert param.global_x_m == None
    assert param.global_y_m == None


def test_set_arguments():
    param = SensorParameters(lon_m=1.0, lat_m=2.0, min_m=3.0, max_m=4.0,
                             reso_deg=5.0, angle_std_scale=6.0, dist_std_rate=7.0)
    
    assert param.INST_LON_M == 1.0
    assert param.INST_LAT_M == 2.0
    assert param.MIN_RANGE_M == 3.0
    assert param.MAX_RANGE_M == 4.0
    assert param.RESO_RAD == np.deg2rad(5.0)
    assert param.ANGLE_STD_SCALE == 6.0
    assert param.DIST_STD_RATE == 7.0


def test_global_pos():
    param = SensorParameters(lon_m=1.0)
    state = MockState()

    param.calculate_global_pos(state)

    assert round(param.get_global_x_m(), 1) == 1.0
    assert round(param.get_global_y_m(), 1) == 2.0


def test_draw_pos():
    figure = plt.figure(figsize=(8, 8))
    axes = figure.add_subplot(111)

    param = SensorParameters()
    state = MockState()
    param.calculate_global_pos(state)

    param.draw_pos(axes, [])