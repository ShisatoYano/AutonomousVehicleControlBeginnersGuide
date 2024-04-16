"""
Unit test of Matrix Lib functions

Author: Shisato Yano
"""

import pytest
import sys
import numpy as np
from pathlib import Path

sys.path.append(str(Path(__file__).absolute().parent) + "/../src/components/common")
from matrix_lib import hom_mat_33


def test_rotation():
    org_tf = hom_mat_33(1.0, 0.0, np.deg2rad(0))
    rot_tf = hom_mat_33(0.0, 0.0, np.deg2rad(90))
    result = rot_tf @ org_tf

    assert round(result[0, 2], 2) == 0.00
    assert round(result[1, 2], 2) == 1.00
    assert round(result[0, 0], 2) == 0.00
    assert round(result[0, 1], 2) == -1.00
    assert round(result[1, 0], 2) == 1.00
    assert round(result[1, 1], 2) == 0.00


def test_translation():
    org_tf = hom_mat_33(1.0, 0.0, np.deg2rad(0))
    rot_tf = hom_mat_33(1.0, -1.0, np.deg2rad(0))
    result = rot_tf @ org_tf

    assert round(result[0, 2], 2) == 2.00
    assert round(result[1, 2], 2) == -1.00
    assert round(result[0, 0], 2) == 1.00
    assert round(result[0, 1], 2) == 0.00
    assert round(result[1, 0], 2) == 0.00
    assert round(result[1, 1], 2) == 1.00


def test_homogeneous_transformation():
    org_tf = hom_mat_33(1.0, 0.0, np.deg2rad(0))
    rot_tf = hom_mat_33(1.0, -1.0, np.deg2rad(90))
    result = rot_tf @ org_tf

    assert round(result[0, 2], 2) == 1.00
    assert round(result[1, 2], 2) == 0.00
    assert round(result[0, 0], 2) == 0.00
    assert round(result[0, 1], 2) == -1.00
    assert round(result[1, 0], 2) == 1.00
    assert round(result[1, 1], 2) == 0.00
