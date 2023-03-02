"""
Unit test of Tire

Author: Shisato Yano
"""

import matplotlib.pyplot as plt
import pytest
import sys
import os

sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../src/vehicle")
from tire import Tire


# mock class
class MockSpecification:
    def __init__(self):
        self.tire_r_m = 1.0
        self.tire_w_m = 1.0
        self.color = 'k'
        self.line_w = 1.0
        self.line_type = '-'


# test instance
spec = MockSpecification()
tire = Tire(spec, 1.0, 1.0)


def test_create_instance():
    assert tire.r_m == 1.0
    assert tire.w_m == 1.0
    assert tire.x_m == 1.0
    assert tire.y_m == 1.0
    assert tire.color == 'k'
    assert tire.line_w == 1.0
    assert tire.line_type == '-'


def test_points():
    assert tire.points.shape[0] == 2
    assert tire.points.shape[1] == 5

    assert tire.points[0, 0] == 2.0
    assert tire.points[0, 1] == 0.0
    assert tire.points[0, 2] == 0.0
    assert tire.points[0, 3] == 2.0
    assert tire.points[0, 4] == 2.0

    assert tire.points[1, 0] == 2.0
    assert tire.points[1, 1] == 2.0
    assert tire.points[1, 2] == 0.0
    assert tire.points[1, 3] == 0.0
    assert tire.points[1, 4] == 2.0


def test_draw_object():
    # plt.clf()
    figure = plt.figure(figsize=(8, 8))
    axes = figure.add_subplot(111)
    tire.draw_object(axes)
