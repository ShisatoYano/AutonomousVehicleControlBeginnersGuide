"""
Unit test of GlobalXYVisualizer

Author: Shisato Yano
"""

import matplotlib.pyplot as plt
import pytest
import sys
import os

sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../src/visualization")
from global_xy_visualizer import GlobalXYVisualizer


# test instance
vis = GlobalXYVisualizer()


# mock class
class MockObject:
    def __init__(self):
        pass

    def update(self, time_interval_s):
        pass

    def draw(self, axes, elems):
        pass


def test_create_instance():
    assert len(vis.objects) == 0
    assert vis.min_lim == 0
    assert vis.max_lim == 30
    assert vis.time_span_s == 10
    assert vis.time_interval_s == 0.1
    assert vis.show_plot == True


def test_draw():
    mock = MockObject()
    vis.add_object(mock)
    assert len(vis.objects) == 1

    vis.not_show_plot()
    assert vis.show_plot == False

    vis.draw()
