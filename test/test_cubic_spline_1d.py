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

X_DATA_VALID = [0, 1, 2, 3, 4]
X_DATA_INVALID = [0, -1, -2, 3, 5]
Y_DATA = [1.7, -6, 5, 6.5, 0.0]


def test_init_value_error():
    with pytest.raises(ValueError) as e:
        CubicSpline(X_DATA_INVALID, Y_DATA)
    
    assert str(e.value) == "X coordinate points must be stored in ascending order"


def test_init_no_error():
    cs = CubicSpline(X_DATA_VALID, Y_DATA)

    assert cs.x_points == X_DATA_VALID
    assert cs.y_points == Y_DATA
