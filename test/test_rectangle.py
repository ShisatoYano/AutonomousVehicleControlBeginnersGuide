"""
Unit test of Rectangle

Author: Shisato Yano
"""

import matplotlib as mpl
mpl.use("Agg")
import matplotlib.pyplot as plt
import pytest
import sys
from pathlib import Path

sys.path.append(str(Path(__file__).absolute().parent) + "/../src/components/detection/l_shape_fitting")
from rectangle import Rectangle

# test instance
rectangle = Rectangle(a=[1, 2, 3, 4],
                      b=[2, 3, 4, 5],
                      c=[3, 4, 5, 6])


def test_initialize():
    assert rectangle.a == [1, 2, 3, 4]
    assert rectangle.b == [2, 3, 4, 5]
    assert rectangle.c == [3, 4, 5, 6]

def test_contour():
    assert rectangle.contour_x == [-1, -1, -1, -1, -1]
    assert rectangle.contour_y == [2, 2, 2, 2, 2]

def test_center():
    assert rectangle.center_x == -1
    assert rectangle.center_y == 2
