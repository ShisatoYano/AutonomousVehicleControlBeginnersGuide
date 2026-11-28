"""
Unit test of cubic_spline.py

Author: Utkarsh Bhangale
GitHub: https://github.com/kragent66-glitch
"""

import math
import sys
from pathlib import Path

import pytest

sys.path.append(str(Path(__file__).absolute().parent) + "/../src/components/course/cubic_spline_course")
from cubic_spline import CubicSpline


def test_position_at_all_points():
    spline = CubicSpline([0, 1, 2, 3], [0, 1, 0, 1])

    assert spline.calculate_position(0) == pytest.approx(0.0)
    assert spline.calculate_position(1) == pytest.approx(1.0)
    assert spline.calculate_position(2) == pytest.approx(0.0)
    assert spline.calculate_position(3) == pytest.approx(1.0)


def test_derivatives_at_last_point():
    # The last point reuses the last segment, so its derivative is the left-hand one
    spline = CubicSpline([0, 1, 2, 3], [0, 1, 0, 1])

    dy = spline.calculate_first_derivative(3)
    ddy = spline.calculate_second_derivative(3)

    assert dy is not None and math.isfinite(dy)
    assert ddy is not None and math.isfinite(ddy)


def test_position_outside_range():
    spline = CubicSpline([0, 1, 2, 3], [0, 1, 0, 1])

    assert spline.calculate_position(-0.5) is None
    assert spline.calculate_position(3.5) is None
