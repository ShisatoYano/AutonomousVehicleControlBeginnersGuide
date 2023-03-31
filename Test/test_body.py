"""
Unit test of Body

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
from body import Body


# mock class
class MockSpecification:
    def __init__(self):
        self.f_edge_m = 1.0
        self.r_edge_m = 1.0
        self.width_m = 1.0
        self.color = 'k'
        self.line_w = 1.0
        self.line_type = '-'


# test instance
spec = MockSpecification()
body = Body(spec)


def test_create_instance():
    assert hasattr(body, "spec") == True
    assert hasattr(body, "array") == True
    assert body.spec != None
    assert body.array != None


def test_draw():
    figure = plt.figure(figsize=(8, 8))
    axes = figure.add_subplot(111)
    assert body.draw(axes, np.array([[0.0], [0.0], [0.0]])) != None
