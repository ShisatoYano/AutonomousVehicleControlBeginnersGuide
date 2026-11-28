"""
Unit test of LqrController's Riccati solution and control gain

Author: Pham Tran Gia Hung
"""

import numpy as np
import pytest
import scipy.linalg as la
import sys
from pathlib import Path
from types import SimpleNamespace

sys.path.append(str(Path(__file__).absolute().parent) + "/../src/components/control/lqr")
from lqr_controller import LqrController

WHEEL_BASE_M = 2.5

# The iteration stops once one step changes X by under THRESHOLD (0.01), about
# 0.1 from the true solution here, so compare within 1% of the largest entry.
REL_TOL = 0.01


def _controller():
    return LqrController(SimpleNamespace(wheel_base_m=WHEEL_BASE_M))


def _state_matrices(speed_mps, time_s):
    # The A and B that LqrController._calculate_control_input builds
    A = np.zeros((5, 5))
    A[0, 0] = 1.0
    A[0, 1] = time_s
    A[1, 2] = speed_mps
    A[2, 2] = 1.0
    A[2, 3] = time_s
    A[4, 4] = 1.0
    B = np.zeros((5, 2))
    B[3, 0] = speed_mps / WHEEL_BASE_M
    B[4, 1] = time_s
    return A, B


def _scipy_solution(controller, A, B):
    return la.solve_discrete_are(A, B, controller.WEIGHT_MAT_Q, controller.WEIGHT_MAT_R)


CASES = [(5.0, 0.1), (10.0, 0.1), (2.0, 0.05)]


@pytest.mark.parametrize(("speed_mps", "time_s"), CASES)
def test_riccati_solution_matches_scipy_solve_discrete_are(speed_mps, time_s):
    controller = _controller()
    A, B = _state_matrices(speed_mps, time_s)

    X = controller._solve_riccati_equation(A, B)
    expected = _scipy_solution(controller, A, B)

    assert np.abs(X - expected).max() < REL_TOL * np.abs(expected).max()


@pytest.mark.parametrize(("speed_mps", "time_s"), CASES)
def test_riccati_solution_is_symmetric_positive_definite(speed_mps, time_s):
    A, B = _state_matrices(speed_mps, time_s)

    X = _controller()._solve_riccati_equation(A, B)

    assert np.allclose(X, X.T)
    assert np.all(np.linalg.eigvalsh(X) > 0.0)


@pytest.mark.parametrize(("speed_mps", "time_s"), CASES)
def test_control_gain_matches_gain_from_scipy_solution(speed_mps, time_s):
    controller = _controller()
    A, B = _state_matrices(speed_mps, time_s)
    R = controller.WEIGHT_MAT_R

    gain = controller._calculate_control_gain(A, B)
    X = _scipy_solution(controller, A, B)
    expected = la.inv(B.T @ X @ B + R) @ (B.T @ X @ A)

    assert gain.shape == (2, 5)
    assert np.abs(gain - expected).max() < REL_TOL * np.abs(expected).max()
