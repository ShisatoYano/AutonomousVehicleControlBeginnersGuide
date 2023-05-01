"""
Unit test of GlobalXYVisualizer

Author: Shisato Yano
"""

import matplotlib.pyplot as plt
import pytest
import sys
from pathlib import Path

sys.path.append(str(Path(__file__).absolute().parent) + "/../src/visualization")
from global_xy_visualizer import GlobalXYVisualizer


# mock class
class MockObject:
    def __init__(self):
        pass

    def update(self, time_interval_s):
        pass

    def draw(self, axes, elems):
        pass


def test_initialize():
    vis = GlobalXYVisualizer()
    assert len(vis.objects) == 0
    assert vis.x_min == 0
    assert vis.x_max == 30
    assert vis.y_min == -15
    assert vis.y_max == 15
    assert vis.time_span_s == 10
    assert vis.time_interval_s == 0.1
    assert vis.save_gif_name == None
    assert vis.show_plot == True


def test_set_parameters():
    test_gif_name = "test.gif"

    vis = GlobalXYVisualizer(x_min=-10, x_max=10, y_min=-10, y_max=10,
                             time_span_s=20, time_interval_s=0.05,
                             save_gif_name=test_gif_name)
    assert vis.x_min == -10
    assert vis.x_max == 10
    assert vis.y_min == -10
    assert vis.y_max == 10
    assert vis.time_span_s == 20
    assert vis.time_interval_s == 0.05
    assert vis.save_gif_name == test_gif_name
    assert vis.show_plot == True


def test_draw():
    mock = MockObject()
    vis = GlobalXYVisualizer()

    vis.add_object(mock)
    assert len(vis.objects) == 1

    vis.not_show_plot()
    assert vis.show_plot == False

    vis.draw()
