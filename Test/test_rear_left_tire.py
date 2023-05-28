"""
Unit test of RearLeftTire

Author: Shisato Yano
"""

import matplotlib as mpl
mpl.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pytest
import sys
from pathlib import Path

sys.path.append(str(Path(__file__).absolute().parent) + "/../src/vehicle")
from rear_left_tire import RearLeftTire


# mock class
class MockSpecification:
    def __init__(self):
        self.r_len_m = 1.0
        self.axle_half_m = 1.0
        self.tire_r_m = 1.0
        self.tire_w_m = 1.0
        self.color = 'k'
        self.line_w = 1.0
        self.line_type = '-'


# test instance
spec = MockSpecification()
tire = RearLeftTire(spec)


def test_create_instance():
    assert hasattr(tire, "spec") == True
    assert hasattr(tire, "array") == True
    assert tire.spec != None
    assert tire.array != None
    assert tire.offset_x_m == -1.0
    assert tire.offset_y_m == 1.0


def test_draw():
    plt.clf()
    plt.close()
    
    figure = plt.figure(figsize=(8, 8))
    axes = figure.add_subplot(111)
    tire.draw(axes, np.array([[0.0], [0.0], [0.0]]), [])
