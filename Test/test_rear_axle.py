"""
Unit test of RearAxle

Author: Shisato Yano
"""

import matplotlib.pyplot as plt
import numpy as np
import pytest
import sys
import os

sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../src/vehicle")
from rear_axle import RearAxle


# mock class
class MockSpecification:
    def __init__(self):
        self.r_len_m = -1.0
        self.axle_half_m = 1.0
        self.color = 'k'
        self.line_w = 1.0
        self.line_type = '-'


# test instance
spec = MockSpecification()
axle = RearAxle(spec)


def test_create_instance():
    assert hasattr(axle, "spec") == True
    assert hasattr(axle, "array") == True
    assert axle.spec != None
    assert axle.array != None
    assert axle.offset_x_m == -1.0
    assert axle.offset_y_m == 1.0


def test_draw():
    figure = plt.figure(figsize=(8, 8))
    axes = figure.add_subplot(111)
    assert axle.draw(axes, np.array([[0.0], [0.0], [0.0]])) != None
