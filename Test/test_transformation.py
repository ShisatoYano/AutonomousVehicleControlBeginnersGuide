"""
Unit test of Transformation

Author: Shisato Yano
"""

import pytest
import sys
import os
import numpy as np

sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../src/common")
from transformation import Transformation


def test_rotation():
    points = np.array([[1.0], [0.0]])

    rotated_points = Transformation.rotation(points, np.deg2rad(90))
    assert int(rotated_points[0, 0]) == 0.0
    assert int(rotated_points[1, 0]) == 1.0

    rotated_points = Transformation.rotation(points, np.deg2rad(180))
    assert int(rotated_points[0, 0]) == -1.0
    assert int(rotated_points[1, 0]) == 0.0

    rotated_points = Transformation.rotation(points, np.deg2rad(270))
    assert int(rotated_points[0, 0]) == 0.0
    assert int(rotated_points[1, 0]) == -1.0
