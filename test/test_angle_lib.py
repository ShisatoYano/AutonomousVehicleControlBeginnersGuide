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
    assert pi_to_pi(3 * np.pi) == -np.pi
    assert pi_to_pi(4 * np.pi) == 0.0
    assert pi_to_pi(5 * np.pi / 2) == np.pi / 2
    assert pi_to_pi(-3 * np.pi) == -np.pi
    assert pi_to_pi(-4 * np.pi) == 0.0
    assert pi_to_pi(-5 * np.pi / 2) == -np.pi / 2


def test_numpy_array():
    angles = np.array([0.0, 3 * np.pi, -5 * np.pi / 2, 4 * np.pi])
    expected = np.array([0.0, -np.pi, -np.pi / 2, 0.0])
    np.testing.assert_allclose(pi_to_pi(angles), expected)
    np.testing.assert_allclose(
        zero_to_2pi(np.array([0.0, -np.pi / 2, 3 * np.pi])),
        np.array([0.0, 3 * np.pi / 2, np.pi]),
    )
