"""
Unit test of Chassis

Author: Shisato Yano
"""

import matplotlib.pyplot as plt
import pytest
import sys
import os

sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../src/vehicle")
from chassis import Chassis


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
chassis = Chassis(spec)


def test_create_instance():
    assert chassis.f_len_m == 1.0
    assert chassis.r_len_m == 1.0
    assert chassis.color == 'k'
    assert chassis.line_w == 1.0
    assert chassis.line_type == '-'


def test_points():
    assert chassis.points.shape[0] == 2
    assert chassis.points.shape[1] == 2

    assert chassis.points[0, 0] == 1.0
    assert chassis.points[0, 1] == -1.0

    assert chassis.points[1, 0] == 0.0
    assert chassis.points[1, 1] == 0.0


def test_draw():
    plt.clf()
    figure = plt.figure(figsize=(8, 8))
    axes = figure.add_subplot(111)
    chassis.draw(axes)
