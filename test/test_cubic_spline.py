"""
Unit test of CubicSpline

Author: Mugunth007
"""

import numpy as np
import pytest
import sys
from pathlib import Path

sys.path.append(str(Path(__file__).absolute().parent) + "/../src/components/course/cubic_spline_course")
from cubic_spline import CubicSpline


def test_invalid_x_points_order_raises():
    with pytest.raises(ValueError):
        CubicSpline([0.0, 2.0, 1.0], [0.0, 1.0, 2.0])


def test_calculate_position_passes_through_points():
    x = [0.0, 1.0, 2.0, 3.0]
    y = [1.0, 3.0, 2.0, 4.0]
    spline = CubicSpline(x, y)

    # Check that each given point matches (leaving out the last point per #321)
    for i in range(len(x) - 1):
        pos = spline.calculate_position(x[i])
        assert pos is not None
        assert np.isclose(pos, y[i])


def test_out_of_bounds_returns_none():
    spline = CubicSpline([0.0, 1.0, 2.0], [0.0, 1.0, 0.0])
    assert spline.calculate_position(-1.0) is None
    assert spline.calculate_position(3.0) is None
    assert spline.calculate_first_derivative(-1.0) is None
    assert spline.calculate_second_derivative(-1.0) is None


def test_derivatives_smooth_at_joints():
    x = [0.0, 1.0, 2.0, 3.0]
    y = [0.0, 1.0, 0.0, 1.0]
    spline = CubicSpline(x, y)
    eps = 1e-6

    # At each inner joint, first and second derivatives from left and right segment must agree
    for i in range(1, len(x) - 1):
        joint_x = x[i]
        dy_left = spline.calculate_first_derivative(joint_x - eps)
        dy_right = spline.calculate_first_derivative(joint_x + eps)
        assert np.isclose(dy_left, dy_right, atol=1e-4)

        ddy_left = spline.calculate_second_derivative(joint_x - eps)
        ddy_right = spline.calculate_second_derivative(joint_x + eps)
        assert np.isclose(ddy_left, ddy_right, atol=1e-4)


def test_straight_line_derivatives():
    # Points on y = 2.0 * x + 1.0
    x = [0.0, 1.0, 2.0, 3.0]
    y = [1.0, 3.0, 5.0, 7.0]
    spline = CubicSpline(x, y)

    # In a straight line, second derivative is 0 and first derivative equals slope everywhere
    for eval_x in [0.0, 0.5, 1.0, 1.5, 2.0]:
        dy = spline.calculate_first_derivative(eval_x)
        ddy = spline.calculate_second_derivative(eval_x)
        assert dy is not None and np.isclose(dy, 2.0, atol=1e-6)
        assert ddy is not None and np.isclose(ddy, 0.0, atol=1e-6)
