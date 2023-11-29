"""
Unit test of ExtendedKalmanFilterLocalizer

Author: Shisato Yano
"""

import matplotlib as mpl
mpl.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pytest
import sys
from pathlib import Path

sys.path.append(str(Path(__file__).absolute().parent) + "/../src/components/localization/kalman_filter")
from extended_kalman_filter_localizer import ExtendedKalmanFilterLocalizer


class MockState:
    def __init__(self):
        pass
    
    def get_x_m(self):
        return 0.0

    def get_y_m(self):
        return 0.0
    
    def get_yaw_rad(self):
        return 0.0

    def get_speed_mps(self):
        return 0.0


def test_initialization():
    localizer = ExtendedKalmanFilterLocalizer()

    assert localizer.state[0, 0] == 0
    assert localizer.state[1, 0] == 0
    assert localizer.state[2, 0] == 0
    assert localizer.state[3, 0] == 0

    assert localizer.cov_mat[0, 0] == 1
    assert localizer.cov_mat[1, 1] == 1
    assert localizer.cov_mat[2, 2] == 1
    assert localizer.cov_mat[3, 3] == 1

    assert localizer.INPUT_NOISE_VAR_MAT[0, 0] == (0.2**2)
    assert localizer.INPUT_NOISE_VAR_MAT[1, 1] == (np.deg2rad(10)**2)

    assert localizer.OBSRV_NOISE_VAR_MAT[0, 0] == (1.0**2)
    assert localizer.OBSRV_NOISE_VAR_MAT[1, 1] == (1.0**2)

    assert localizer.JACOB_H[0, 0] == 1
    assert localizer.JACOB_H[1, 1] == 1

    assert localizer.DRAW_COLOR == 'r'


def test_update():
    localizer = ExtendedKalmanFilterLocalizer(accel_noise=0.0, yaw_rate_noise=0.0,
                                              obsrv_x_noise=0.0, obsrv_y_noise=0.0)
    
    state = MockState()
    est_state = localizer.update(state, 0.0, 0.0, 0.0, np.array([[0.0],[0.0]]))
    assert est_state[0, 0] == 0.0
    assert est_state[1, 0] == 0.0
    assert est_state[2, 0] == 0.0
    assert est_state[3, 0] == 0.0


def test_draw():
    localizer = ExtendedKalmanFilterLocalizer()
    plt.clf()
    plt.close()
    figure = plt.figure(figsize=(8, 8))
    axes = figure.add_subplot(111)
    localizer.draw(axes, [], np.array([[0.0],[0.0]]))
