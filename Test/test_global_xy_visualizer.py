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


class MockTimeParameters:
    def __init__(self):
        self.span_sec = 10
        self.interval_sec = 0.1
        self.interval_msec = 100
        self.frame_num = 101
    
    def get_interval_sec(self):
        return self.interval_sec
    
    def get_interval_msec(self):
        return self.interval_msec
    
    def get_frame_num(self):
        return self.frame_num
    
    def current_sec(self, index):
        return self.interval_sec * index
    
    def simulation_finished(self, index):
        return (self.interval_sec * index >= self.span_sec)


def test_initialize():
    x_lim, y_lim = MockMinMax(), MockMinMax()
    vis = GlobalXYVisualizer(x_lim, y_lim, MockTimeParameters())
    assert len(vis.objects) == 0
    assert vis.x_lim != None
    assert vis.y_lim != None
    assert vis.time_params != None
    assert vis.save_gif_name == None
    assert vis.show_plot == True


def test_gif_name():
    test_gif_name = "test.gif"

    x_lim, y_lim = MockMinMax(), MockMinMax()
    vis = GlobalXYVisualizer(x_lim, y_lim, MockTimeParameters(),
                             save_gif_name=test_gif_name)
    assert vis.save_gif_name == test_gif_name


def test_draw():
    mock = MockObject()

    x_lim, y_lim = MockMinMax(), MockMinMax()
    vis = GlobalXYVisualizer(x_lim, y_lim, MockTimeParameters())

    vis.add_object(mock)
    assert len(vis.objects) == 1

    vis.not_show_plot()
    assert vis.show_plot == False

    vis.draw()
