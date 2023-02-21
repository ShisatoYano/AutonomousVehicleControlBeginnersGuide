"""
Unit test of GlobalXYVisualizer

Author: Shisato Yano
"""

import pytest
import sys
import os

sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../src/visualization")
from global_xy_visualizer import GlobalXYVisualizer


def test_initialize():
    vis = GlobalXYVisualizer()

    assert vis.o_min == -5
