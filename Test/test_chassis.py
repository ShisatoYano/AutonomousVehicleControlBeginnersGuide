"""
Unit test of Chassis

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
    assert hasattr(chassis, "spec") == True
    assert hasattr(chassis, "array") == True
    assert chassis.spec != None
    assert chassis.array != None


def test_draw():
    figure = plt.figure(figsize=(8, 8))
    axes = figure.add_subplot(111)
    chassis.draw(axes, np.array([[0.0], [0.0], [0.0]]), [])
