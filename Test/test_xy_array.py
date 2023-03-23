"""
Unit test of XYArray

Author: Shisato Yano
"""

import pytest
import sys
import os
import numpy as np

sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../src/array")
from xy_array import XYArray


# test instance
input_data = np.array([[1.0], [0.0]])
array = XYArray(input_data)


def test_data():
    data = array.get_data()
    assert round(data[0, 0], 1) == 1.0
    assert round(data[1, 0], 1) == 0.0


def test_homogeneous_transformation():
    transformed_array_1 = array.homogeneous_transformation(1.0, -1.0, np.deg2rad(90))
    data_1 = transformed_array_1.get_data()
    assert round(data_1[0, 0], 1) == 1.0
    assert round(data_1[1, 0], 1) == 0.0

    transformed_array_2 = array.homogeneous_transformation(1.0, -1.0, np.deg2rad(180))
    data_2 = transformed_array_2.get_data()
    assert round(data_2[0, 0], 1) == 0.0
    assert round(data_2[1, 0], 1) == -1.0

    transformed_array_3 = array.homogeneous_transformation(1.0, -1.0, np.deg2rad(270))
    data_3 = transformed_array_3.get_data()
    assert round(data_3[0, 0], 1) == 1.0
    assert round(data_3[1, 0], 1) == -2.0
