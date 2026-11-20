"""
Unit test of Angle Lib functions

Author: Shisato Yano
"""

import pytest
import sys
import numpy as np
from pathlib import Path

sys.path.append(str(Path(__file__).absolute().parent) + "/../src/components/common")
from angle_lib import pi_to_pi, zero_to_2pi


def test_lower_than_pi():
    assert pi_to_pi(0.0) == 0.0
    assert pi_to_pi(np.pi / 2) == np.pi / 2
    assert pi_to_pi(np.pi) == -np.pi


def test_greater_than_pi():
    assert pi_to_pi(3 * np.pi / 2) == -np.pi / 2
    assert pi_to_pi(2 * np.pi) == 0.0


def test_lower_than_negative_pi():
    assert pi_to_pi(-3 * np.pi / 2) == np.pi / 2
    assert pi_to_pi(-2 * np.pi) == 0.0


def test_greater_than_negative_pi():
    assert pi_to_pi(-np.pi / 2) == -np.pi / 2
    assert pi_to_pi(-np.pi) == -np.pi


def test_angles_beyond_three_pi():
    assert pi_to_pi(5 * np.pi / 2) == pytest.approx(np.pi / 2)
    assert pi_to_pi(7 * np.pi / 2) == pytest.approx(-np.pi / 2)
    assert pi_to_pi(-7 * np.pi / 2) == pytest.approx(np.pi / 2)
    assert pi_to_pi(100 * np.pi + 0.1) == pytest.approx(0.1)
    assert pi_to_pi(-100 * np.pi - 0.1) == pytest.approx(-0.1)


def test_result_is_always_in_range():
    for angle in np.linspace(-50.0, 50.0, 1001):
        assert -np.pi <= pi_to_pi(angle) < np.pi


def test_numpy_array_input():
    angles = np.array([0.0, 3 * np.pi / 2, 7 * np.pi / 2, -5 * np.pi / 2])
    expected = np.array([0.0, -np.pi / 2, -np.pi / 2, -np.pi / 2])
    np.testing.assert_allclose(pi_to_pi(angles), expected, atol=1e-12)


def test_zero_to_2pi():
    assert zero_to_2pi(0.0) == 0.0
    assert zero_to_2pi(-np.pi / 2) == pytest.approx(3 * np.pi / 2)
    assert zero_to_2pi(5 * np.pi) == pytest.approx(np.pi)
    assert zero_to_2pi(-7 * np.pi / 2) == pytest.approx(np.pi / 2)
    angles = np.array([-np.pi / 2, 2 * np.pi, 9 * np.pi / 2])
    np.testing.assert_allclose(zero_to_2pi(angles), [3 * np.pi / 2, 0.0, np.pi / 2], atol=1e-12)
