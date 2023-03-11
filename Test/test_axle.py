"""
Unit test of Axle

Author: Shisato Yano
"""

import matplotlib.pyplot as plt
import numpy as np
import pytest
import sys
import os

sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../src/vehicle")
from axle import Axle


# mock class
class MockSpecification:
    def __init__(self):
        self.color = 'k'
        self.line_w = 1.0
        self.line_type = '-'


# test instance
spec = MockSpecification()
axle = Axle(spec, 1.0, 1.0)


def test_create_instance():
    assert axle.x_m == 1.0
    assert axle.y_m == 1.0
    assert axle.color == 'k'
    assert axle.line_w == 1.0
    assert axle.line_type == '-'


def test_points():
    assert axle.points.shape[0] == 2
    assert axle.points.shape[1] == 2

    assert axle.points[0, 0] == 0.0
    assert axle.points[0, 1] == 0.0

    assert axle.points[1, 0] == 1.0
    assert axle.points[1, 1] == -1.0


def test_draw():
    figure = plt.figure(figsize=(8, 8))
    axes = figure.add_subplot(111)
    axle.draw(axes, np.array([[0.0], [0.0], [0.0]]), 0.0)

