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


class MockMinMax:
    def __init__(self):
        self.min = 0
        self.max = 10
    
    def min_value(self):
        return self.min
    
    def max_value(self):
        return self.max


def test_initialize():
    x_lim, y_lim = MockMinMax(), MockMinMax()
    vis = GlobalXYVisualizer(x_lim, y_lim)
    assert len(vis.objects) == 0
    assert vis.x_lim != None
    assert vis.y_lim != None
    assert vis.time_span_s == 10
    assert vis.time_interval_s == 0.1
    assert vis.save_gif_name == None
    assert vis.show_plot == True


def test_set_parameters():
    test_gif_name = "test.gif"

    x_lim, y_lim = MockMinMax(), MockMinMax()
    vis = GlobalXYVisualizer(x_lim, y_lim,
                             time_span_s=20, time_interval_s=0.05,
                             save_gif_name=test_gif_name)
    assert vis.x_lim.min_value() == 0
    assert vis.x_lim.max_value() == 10
    assert vis.y_lim.min_value() == 0
    assert vis.y_lim.max_value() == 10
    assert vis.time_span_s == 20
    assert vis.time_interval_s == 0.05
    assert vis.save_gif_name == test_gif_name
    assert vis.show_plot == True


def test_draw():
    mock = MockObject()

    x_lim, y_lim = MockMinMax(), MockMinMax()
    vis = GlobalXYVisualizer(x_lim, y_lim)

    vis.add_object(mock)
    assert len(vis.objects) == 1

    vis.not_show_plot()
    assert vis.show_plot == False

    vis.draw()
