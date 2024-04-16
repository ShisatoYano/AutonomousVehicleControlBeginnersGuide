"""
Unit test of UkfParamsCalibrator

Author: Shisato Yano
"""

import matplotlib as mpl
mpl.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pytest
import sys
from pathlib import Path

sys.path.append(str(Path(__file__).absolute().parent) + "/../src/components/sensors/lidar")
sys.path.append(str(Path(__file__).absolute().parent) + "/../src/components/common")
from ukf_ext_params_calibrator import UkfExtParamsCalibrator
from matrix_lib import hom_mat_33


class MockState:
    def __init__(self):
        pass
    
    def x_y_yaw(self):
        return np.array([[0.0], [0.0], [0.0]])


def test_initialization():
    calibrator = UkfExtParamsCalibrator()

    assert calibrator.DIM_NUM == 3
    assert calibrator.ALPHA == 0.001
    assert calibrator.BETA == 2
    assert calibrator.KAPPA == 0

    assert calibrator.LAMBDA == -2.999997
    assert calibrator.STATE_WEIGHTS.shape[0] == 1
    assert calibrator.STATE_WEIGHTS.shape[1] == 7
    assert calibrator.COV_WEIGHTS.shape[0] == 1
    assert calibrator.COV_WEIGHTS.shape[1] == 7
    assert round(calibrator.GAMMA, 4) == 0.0017

    assert calibrator.SYS_NOISE[0, 0] == (0.1 ** 2)
    assert calibrator.SYS_NOISE[1, 1] == (0.1 ** 2)
    assert calibrator.SYS_NOISE[2, 2] == (np.deg2rad(0.1) ** 2)
    assert calibrator.OBV_NOISE[0, 0] == (0.5 ** 2)
    assert calibrator.OBV_NOISE[1, 1] == (0.5 ** 2)
    assert calibrator.OBV_NOISE[2, 2] == (np.deg2rad(0.5) ** 2)

    assert calibrator.state[0, 0] == 0.0
    assert calibrator.state[1, 0] == 0.0
    assert calibrator.state[2, 0] == 0.0
    assert calibrator.cov[0, 0] == 1.0
    assert calibrator.cov[1, 1] == 1.0
    assert calibrator.cov[2, 2] == 1.0


def test_calibrate_extrinsic_params():
    calibrator = UkfExtParamsCalibrator()
    
    sensor_odom_tf = hom_mat_33(0.0, 0.0, 0.0)
    vehicle_odom_tf = hom_mat_33(0.0, 0.0, 0.0)
    calibrator.calibrate_extrinsic_params(sensor_odom_tf, vehicle_odom_tf)

    assert calibrator.state[0, 0] == 0.0
    assert calibrator.state[1, 0] == 0.0
    assert calibrator.state[2, 0] == 0.0
    assert calibrator.cov[0, 0] != 1.0
    assert calibrator.cov[1, 1] != 1.0
    assert calibrator.cov[2, 2] != 1.0


def test_draw_calib_result():
    calibrator = UkfExtParamsCalibrator()

    plt.clf()
    plt.close()
    figure = plt.figure(figsize=(8, 8))
    axes = figure.add_subplot(111)

    calibrator.draw_calib_result(axes, [], MockState(), 0.0, 0.0, 0.0)
