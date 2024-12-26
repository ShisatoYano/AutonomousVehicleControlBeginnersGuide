"""
Unit test of cubic_pline_2d_plot.py

Author: Shisato Yano
"""

import matplotlib as mpl
mpl.use("Agg")
from pathlib import Path
import sys

sys.path.append(str(Path(__file__).absolute().parent) + "/../src/simulations/course/cubic_spline")
import cubic_spline_2d_plot


def test_simulation():
    cubic_spline_2d_plot.show_plot = False

    cubic_spline_2d_plot.main()
