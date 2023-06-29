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
from pathlib import Path

sys.path.append(str(Path(__file__).absolute().parent) + "/../src/components/vehicle")
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
    plt.clf()
    plt.close()
    figure = plt.figure(figsize=(8, 8))
    axes = figure.add_subplot(111)
    body.draw(axes, np.array([[0.0], [0.0], [0.0]]), [])
