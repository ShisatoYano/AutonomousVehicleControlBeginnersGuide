"""
cubic_spline_plot.py

Author: Shisato Yano
"""

# import path setting
import numpy as np
import sys
import matplotlib.pyplot as plt
from pathlib import Path

abs_dir_path = str(Path(__file__).absolute().parent)
relative_path = "/../../../components/"

sys.path.append(abs_dir_path + relative_path + "course/cubic_spline")

# import component module
from cubic_spline import CubicSpline


# flag to show plot figure
# when executed as unit test, this flag is set as false
show_plot = True
