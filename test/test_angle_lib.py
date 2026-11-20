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


def test_beyond_three_pi():
    # wrapping once is not enough for these: 7pi/2 - 2pi is still 3pi/2
    assert pi_to_pi(7 * np.pi / 2) == pytest.approx(-np.pi / 2)
    assert pi_to_pi(-7 * np.pi / 2) == pytest.approx(np.pi / 2)
    assert pi_to_pi(10 * np.pi + 0.5) == pytest.approx(0.5)
    assert pi_to_pi(-10 * np.pi - 0.5) == pytest.approx(-0.5)


def test_numpy_array():
    angles = np.array([0.0, 3 * np.pi / 2, -3 * np.pi / 2, 7 * np.pi / 2, 10 * np.pi + 0.5])
    expected = np.array([0.0, -np.pi / 2, np.pi / 2, -np.pi / 2, 0.5])
    assert np.allclose(pi_to_pi(angles), expected)


def test_result_is_always_within_range():
    angles = np.linspace(-20 * np.pi, 20 * np.pi, 1001)
    wrapped = pi_to_pi(angles)
    assert np.all(wrapped >= -np.pi) and np.all(wrapped < np.pi)
    assert np.allclose(np.sin(wrapped), np.sin(angles))
    assert np.allclose(np.cos(wrapped), np.cos(angles))


def test_zero_to_2pi():
    assert zero_to_2pi(0.0) == 0.0
    assert zero_to_2pi(np.pi) == np.pi
    assert zero_to_2pi(-np.pi / 2) == pytest.approx(3 * np.pi / 2)
    assert zero_to_2pi(2 * np.pi) == 0.0
    assert zero_to_2pi(-7 * np.pi / 2) == pytest.approx(np.pi / 2)
    angles = np.linspace(-20 * np.pi, 20 * np.pi, 1001)
    wrapped = zero_to_2pi(angles)
    assert np.all(wrapped >= 0.0) and np.all(wrapped < 2 * np.pi)
    assert np.allclose(np.sin(wrapped), np.sin(angles))
