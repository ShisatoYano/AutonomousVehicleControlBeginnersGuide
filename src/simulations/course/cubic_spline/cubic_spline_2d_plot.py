"""
cubic_spline_2d_plot.py

Author: Shisato Yano
"""

# import path setting
import numpy as np
import sys
import matplotlib.pyplot as plt
from pathlib import Path

abs_dir_path = str(Path(__file__).absolute().parent)
relative_path = "/../../../components/"

sys.path.append(abs_dir_path + relative_path + "course/cubic_spline_course")

# import component module
from cubic_spline_2d import CubicSpline2D


# flag to show plot figure
# when executed as unit test, this flag is set as false
show_plot = True


def main():
    """
    Main process function
    """

    x_points = [-2.5, 0.0, 2.5, 5.0, 7.5, 3.0, -1.0]
    y_points = [0.7, -6, 5, 6.5, 0.0, 5.0, -2.0]

    ds = 0.1 # distance between 2 interpolated points

    cs = CubicSpline2D(x_points, y_points)
    s = np.arange(0, cs.s[-1], ds)


if __name__ == "__main__":
    main()
