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

    xs, ys, yaws, curvs = [], [], [], []
    for i_s in s:
        i_x, i_y = cs.calc_interpolated_xy(i_s)
        xs.append(i_x)
        ys.append(i_y)
        yaws.append(cs.calc_yaw_angle(i_s))
    
    plt.subplots(1)
    plt.plot(x_points, y_points, "xb", label="Input points")
    plt.plot(xs, ys, "-r", label="Cubic spline path")
    plt.grid(True)
    plt.axis("equal")
    plt.xlabel("X[m]")
    plt.ylabel("Y[m]")
    plt.legend()
    if show_plot: plt.savefig("cubic_spline_2d.png")

    plt.subplots(1)
    plt.plot(s, [np.rad2deg(yaw) for yaw in yaws], "-r", label="Yaw angle")
    plt.grid(True)
    plt.xlabel("Line length[m]")
    plt.ylabel("Yaw angle[deg]")
    plt.legend()
    if show_plot: plt.savefig("cubic_spline_yaw_angle.png")


if __name__ == "__main__":
    main()
