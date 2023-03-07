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
    assert round(rotated_points[0, 0], 1) == 0.0
    assert round(rotated_points[1, 0], 1) == 1.0

    rotated_points = Transformation.rotation(points, np.deg2rad(180))
    assert round(rotated_points[0, 0], 1) == -1.0
    assert round(rotated_points[1, 0], 1) == 0.0

    rotated_points = Transformation.rotation(points, np.deg2rad(270))
    assert round(rotated_points[0, 0], 1) == 0.0
    assert round(rotated_points[1, 0], 1) == -1.0


def test_translation():
    points = np.array([[1.0], [0.0]])

    translated_points = Transformation.translation(points, 1.0, 0.0)
    assert round(translated_points[0, 0], 1) == 2.0
    assert round(translated_points[1, 0], 1) == 0.0

    translated_points = Transformation.translation(points, 0.0, 1.0)
    assert round(translated_points[0, 0], 1) == 1.0
    assert round(translated_points[1, 0], 1) == 1.0

    translated_points = Transformation.translation(points, -1.0, -1.0)
    assert round(translated_points[0, 0], 1) == 0.0
    assert round(translated_points[1, 0], 1) == -1.0


def test_homogeneous_transformation():
    points = np.array([[1.0], [0.0]])

    transformed_points = Transformation.homogeneous_transformation(points, np.array([[1.0], [-1.0], [np.deg2rad(90)]]))
    assert round(transformed_points[0, 0], 1) == 1.0
    assert round(transformed_points[1, 0], 1) == 0.0

    transformed_points = Transformation.homogeneous_transformation(points, np.array([[1.0], [-1.0], [np.deg2rad(180)]]))
    assert round(transformed_points[0, 0], 1) == 0.0
    assert round(transformed_points[1, 0], 1) == -1.0

    transformed_points = Transformation.homogeneous_transformation(points, np.array([[1.0], [-1.0], [np.deg2rad(270)]]))
    assert round(transformed_points[0, 0], 1) == 1.0
    assert round(transformed_points[1, 0], 1) == -2.0
