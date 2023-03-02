"""
Unit test of Body

Author: Shisato Yano
"""

import matplotlib.pyplot as plt
import pytest
import sys
import os

sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../src/vehicle")
from body import Body


# mock class
class MockSpecification:
    def __init__(self):
        self.f_len_m = 1.0
        self.r_len_m = 1.0
        self.color = 'k'
        self.line_w = 1.0
        self.line_type = '-'


# test instance
spec = MockSpecification()
body = Body(spec)


def test_create_instance():
    assert body.f_len_m == 0.75
    assert body.r_len_m == 0.75
    assert body.tread_m == 0.75
    assert body.f_edge_m == 1.50
    assert body.r_edge_m == 1.50
    assert body.width_m == 0.75
    assert body.color == 'k'
    assert body.line_w == 1.0
    assert body.line_type == '-'


def test_points():
    assert body.points.shape[0] == 2
    assert body.points.shape[1] == 5

    assert body.points[0, 0] == 1.50
    assert body.points[0, 1] == -1.50
    assert body.points[0, 2] == -1.50
    assert body.points[0, 3] == 1.50
    assert body.points[0, 4] == 1.50

    assert body.points[1, 0] == 0.75
    assert body.points[1, 1] == 0.75
    assert body.points[1, 2] == -0.75
    assert body.points[1, 3] == -0.75
    assert body.points[1, 4] == 0.75


def test_draw():
    plt.clf()
    figure = plt.figure(figsize=(8, 8))
    axes = figure.add_subplot(111)
    body.draw_object(axes)
