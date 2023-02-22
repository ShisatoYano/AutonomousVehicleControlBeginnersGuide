"""
Unit test of GlobalXYVisualizer

Author: Shisato Yano
"""

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

    def draw(self, a_axes):
        pass


def test_create_instance():
    assert len(vis.o_objects) == 0
    assert vis.o_min == -5
    assert vis.o_max == 5
    assert vis.o_show_plot == True


def test_draw():
    mock = MockObject()
    vis.add_object(mock)
    vis.not_show_plot()
    vis.draw()
    assert len(vis.o_objects) == 1
