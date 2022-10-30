"""
Test code of Linear Kalman Filter 2D

Author: Shisato Yano
"""

import pytest
import sys
import os

# path setting to import module of test target
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../Localization/Sources/kalman_filter")
import linear_kalman_filter_2d


def test_simulation():
    linear_kalman_filter_2d.show_plot = False

    assert linear_kalman_filter_2d.main() == True
