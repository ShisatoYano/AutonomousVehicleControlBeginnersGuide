"""
Unit test of Body

Author: Shisato Yano
"""

import matplotlib.pyplot as plt
import numpy as np
import pytest
import sys
import os

sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../src/vehicle")
from body import Body


# mock class
class MockXYArray:
    def __init__(self):
        pass
    
    def homogeneous_transformation(self, x, y, angle_rad):
        return np.array([[1.0, 0.0], [0.0, 1.0]])


class MockSpecification:
    def __init__(self):
        self.color = 'k'
        self.line_w = 1.0
        self.line_type = '-'


# test instance
array = MockXYArray()
body = Body(array)


def test_create_instance():
    assert hasattr(body, "array") == True


def test_draw():
    figure = plt.figure(figsize=(8, 8))
    axes = figure.add_subplot(111)
    spec = MockSpecification()
    assert body.draw(axes, spec, np.array([[0.0], [0.0], [0.0]])) != None
