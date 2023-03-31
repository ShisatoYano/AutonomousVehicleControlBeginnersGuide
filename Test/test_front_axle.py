"""
Unit test of FrontAxle

Author: Shisato Yano
"""

import matplotlib as mpl
mpl.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pytest
import sys
import os

sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../src/vehicle")
from front_axle import FrontAxle


# mock class
class MockSpecification:
    def __init__(self):
        self.f_len_m = 1.0
        self.axle_half_m = 1.0
        self.color = 'k'
        self.line_w = 1.0
        self.line_type = '-'


# test instance
spec = MockSpecification()
axle = FrontAxle(spec)


def test_create_instance():
    assert hasattr(axle, "spec") == True
    assert hasattr(axle, "array") == True
    assert axle.spec != None
    assert axle.array != None
    assert axle.offset_x_m == 1.0
    assert axle.offset_y_m == 1.0


def test_draw():
    figure = plt.figure(figsize=(8, 8))
    axes = figure.add_subplot(111)
    assert axle.draw(axes, np.array([[0.0], [0.0], [0.0]])) != None
