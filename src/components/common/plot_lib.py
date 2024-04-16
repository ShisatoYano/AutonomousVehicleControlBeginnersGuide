"""
plot_lib.py

Author: Shisato Yano
"""

import sys
import numpy as np
from pathlib import Path
from math import sqrt, cos, sin, atan2, pi

sys.path.append(str(Path(__file__).absolute().parent) + "/../array")
from xy_array import XYArray


def draw_covariance_ellipse(axes, elems, x, y, cov_mat, color='r'):
    """
    Function to draw ellipse of covariance matrix
    axes: Axes object of figure
    elems: List of plot objects
    x: center x position of ellipse
    y: center y position of ellipse
    cov_mat: covariance matrix
    color: color of line
    """
    
    eig_val, eig_vec = np.linalg.eig(cov_mat)
    if eig_val[0] >= eig_val[1]: big_idx, small_idx = 0, 1
    else: big_idx, small_idx = 1, 0
    a, b = sqrt(3.0 * eig_val[big_idx]), sqrt(3.0 * eig_val[small_idx])
    angle = atan2(eig_vec[1, big_idx], eig_vec[0, big_idx])

    t = np.arange(0, 2 * pi + 0.1, 0.1)
    xs = [a * cos(it) for it in t]
    ys = [b * sin(it) for it in t]
    xys = np.array([xs, ys])
    xys_array = XYArray(xys)

    transformed_xys = xys_array.homogeneous_transformation(x, y, angle)
    elip_plot, = axes.plot(transformed_xys.get_x_data(), transformed_xys.get_y_data(), color)
    elems.append(elip_plot)
