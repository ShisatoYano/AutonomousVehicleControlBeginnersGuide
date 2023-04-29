"""
Unit test of SinCurveCourse

Author: Shisato Yano
"""

import matplotlib as mpl
mpl.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pytest
import sys
import os

sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../src/course/sin_curve_course")
from sin_curve_course import SinCurveCourse


# test parameters
X_MIN = 0.0
X_MAX = 50.0
RESOLUTION = 1.0
TARGET_SPEED_KMPH = 20.0

# test instance
course = SinCurveCourse(X_MIN, X_MAX, RESOLUTION, TARGET_SPEED_KMPH)


def test_attributes():
    assert hasattr(course, "x_array") == True
    assert hasattr(course, "y_array") == True
    assert hasattr(course, "speed_array") == True    
