"""
Test of Object Detection simulation by L-Shape Fitting algorithm

Author: Shreyansh Shethia
"""

from pathlib import Path
import sys
import pytest

sys.path.append(str(Path(__file__).absolute().parent) + "/../src/simulations/perception/point_cloud_rectangle_fitting")
import point_cloud_robustLS_rectangle_fitting


def test_simulation():
    point_cloud_robustLS_rectangle_fitting.show_plot = False
    point_cloud_robustLS_rectangle_fitting.main()
