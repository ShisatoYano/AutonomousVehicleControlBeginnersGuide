"""
Unit test of CubicSpline1D

Author: Shisato Yano
"""

import pytest
import sys
import os
from pathlib import Path

sys.path.append(str(Path(__file__).absolute().parent) + "/../src/components/course/cubic_spline_course")
from cubic_spline import CubicSpline


def test_init_value_error():
    x_points = [0, -1, -2, 3, 5]
    y_points = [1.7, -6, 5, 6.5, 0.0]

    with pytest.raises(ValueError) as e:
        CubicSpline(x_points, y_points)
    
    assert str(e.value) == "X coordinate points must be stored in ascending order"
