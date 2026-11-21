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


@pytest.mark.parametrize("angle_rad, expected_rad", [
    (7 * np.pi / 2, -np.pi / 2),
    (-7 * np.pi / 2, np.pi / 2),
    (5 * np.pi, -np.pi),
    (-5 * np.pi, -np.pi),
    (10 * np.pi + 0.5, 0.5),
    (-10 * np.pi - 0.5, -0.5),
])
def test_beyond_three_pi(angle_rad, expected_rad):
    assert pi_to_pi(angle_rad) == pytest.approx(expected_rad)


def test_pi_to_pi_numpy_array():
    angles_rad = np.array([0.0, 3 * np.pi / 2, -3 * np.pi / 2, 7 * np.pi / 2, -7 * np.pi / 2])
    expected_rad = np.array([0.0, -np.pi / 2, np.pi / 2, -np.pi / 2, np.pi / 2])
    result_rad = pi_to_pi(angles_rad)

    assert result_rad.shape == angles_rad.shape
    assert np.allclose(result_rad, expected_rad)
    assert np.all((-np.pi <= result_rad) & (result_rad < np.pi))


@pytest.mark.parametrize("angle_rad, expected_rad", [
    (0.0, 0.0),
    (np.pi, np.pi),
    (2 * np.pi, 0.0),
    (-np.pi / 2, 3 * np.pi / 2),
    (5 * np.pi, np.pi),
    (-7 * np.pi / 2, np.pi / 2),
])
def test_zero_to_2pi(angle_rad, expected_rad):
    assert zero_to_2pi(angle_rad) == pytest.approx(expected_rad)


def test_zero_to_2pi_numpy_array():
    angles_rad = np.array([[-np.pi / 2, 5 * np.pi], [-4 * np.pi, 9 * np.pi / 2]])
    expected_rad = np.array([[3 * np.pi / 2, np.pi], [0.0, np.pi / 2]])
    result_rad = zero_to_2pi(angles_rad)

    assert result_rad.shape == angles_rad.shape
    assert np.allclose(result_rad, expected_rad)
