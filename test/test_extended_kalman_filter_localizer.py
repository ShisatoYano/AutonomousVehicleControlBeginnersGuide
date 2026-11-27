"""
Unit test of ExtendedKalmanFilterLocalizer

Author: Mugunth007
"""

import sys
import numpy as np
import pytest
from pathlib import Path

sys.path.append(str(Path(__file__).absolute().parent) + "/../src/components/localization/kalman_filter")
sys.path.append(str(Path(__file__).absolute().parent) + "/../src/components/state")
from extended_kalman_filter_localizer import ExtendedKalmanFilterLocalizer
from state import State


def _numerical_jacobian_F(state, u, time_s, eps=1e-6):
    """Computes central difference derivative of State.motion_model w.r.t state."""
    jF_num = np.zeros((4, 4))
    for i in range(4):
        state_plus = state.copy()
        state_minus = state.copy()
        state_plus[i, 0] += eps
        state_minus[i, 0] -= eps
        f_plus = State.motion_model(state_plus, u, time_s)
        f_minus = State.motion_model(state_minus, u, time_s)
        jF_num[:, i:i + 1] = (f_plus - f_minus) / (2.0 * eps)
    return jF_num


def _numerical_jacobian_G(state, u, time_s, eps=1e-6):
    """Computes central difference derivative of State.motion_model w.r.t input u."""
    jG_num = np.zeros((4, 2))
    for j in range(2):
        u_plus = u.copy()
        u_minus = u.copy()
        u_plus[j, 0] += eps
        u_minus[j, 0] -= eps
        f_plus = State.motion_model(state, u_plus, time_s)
        f_minus = State.motion_model(state, u_minus, time_s)
        jG_num[:, j:j + 1] = (f_plus - f_minus) / (2.0 * eps)
    return jG_num


@pytest.mark.parametrize("heading", [0.0, np.pi / 4.0, np.pi / 2.0])
@pytest.mark.parametrize("time_s", [0.1, 1.0])
def test_jacobian_F_matches_numerical_derivative(heading, time_s):
    localizer = ExtendedKalmanFilterLocalizer()
    state = np.array([[10.0], [20.0], [heading], [5.0]])
    u = np.array([[1.5], [0.2]])

    jF_analytic = localizer._jacobian_F(state, u, time_s)
    jF_num = _numerical_jacobian_F(state, u, time_s)

    assert np.allclose(jF_analytic, jF_num, atol=1e-6)


@pytest.mark.parametrize("heading", [0.0, np.pi / 4.0, np.pi / 2.0])
@pytest.mark.parametrize("time_s", [0.1, 1.0])
def test_jacobian_G_matches_numerical_derivative(heading, time_s):
    localizer = ExtendedKalmanFilterLocalizer()
    state = np.array([[10.0], [20.0], [heading], [5.0]])
    u = np.array([[1.5], [0.2]])

    jG_analytic = localizer._jacobian_G(state, time_s)
    jG_num = _numerical_jacobian_G(state, u, time_s)

    assert np.allclose(jG_analytic, jG_num, atol=1e-6)
