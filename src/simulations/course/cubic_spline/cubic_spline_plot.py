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

sys.path.append(abs_dir_path + relative_path + "course/cubic_spline_course")

# import component module
from cubic_spline import CubicSpline


# flag to show plot figure
# when executed as unit test, this flag is set as false
show_plot = True


def main():
    """
    Main process function
    """

    x_points = np.arange(5)
    y_points = [1.7, -6, 5, 6.5, 0.0]

    cubic_spline = CubicSpline(x_points, y_points)


if __name__ == "__main__":
    main()
