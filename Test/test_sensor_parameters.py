"""
Unit test of SensorParameters

Author: Shisato Yano
"""

import numpy as np
import pytest
import sys
from pathlib import Path

sys.path.append(str(Path(__file__).absolute().parent) + "/../src/sensors")
from sensor_parameters import SensorParameters


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
