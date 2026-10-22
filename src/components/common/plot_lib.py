"""
plot_lib.py

Author: Shisato Yano
"""

import sys
import numpy as np
from pathlib import Path
from math import atan2, pi

sys.path.append(str(Path(__file__).absolute().parent) + "/../array")
from xy_array import XYArray


def draw_covariance_ellipse(axes, elems, x, y, cov_mat, color='r', **plot_kwargs):
    """
    Function to draw ellipse of covariance matrix
    axes: Axes object of figure
    elems: List of plot objects
    x: center x position of ellipse
    y: center y position of ellipse
    cov_mat: 2x2 covariance matrix of x and y
    color: color of line
    plot_kwargs: other keyword arguments for axes.plot, e.g. linewidth
    """

    # A covariance matrix is symmetric, so eigh applies: its eigenvalues are real
    # and sorted in ascending order, and its eigenvectors are orthonormal.
    eig_val, eig_vec = np.linalg.eigh(cov_mat)
    # rounding can leave an eigenvalue of a nearly singular matrix slightly negative
    eig_val = np.maximum(eig_val, 0.0)

    # The scale factor 3.0 draws the ellipse on which the squared Mahalanobis
    # distance from the center is 3: its half axes are sqrt(3) standard deviations
    # along the eigenvectors, and it holds 1 - exp(-3 / 2), about 78%, of a 2D
    # Gaussian distribution.
    a, b = np.sqrt(3.0 * eig_val[1]), np.sqrt(3.0 * eig_val[0])
    angle = atan2(eig_vec[1, 1], eig_vec[0, 1])

    t = np.arange(0, 2 * pi + 0.1, 0.1)
    xys_array = XYArray(np.array([a * np.cos(t), b * np.sin(t)]))

    transformed_xys = xys_array.homogeneous_transformation(x, y, angle)
    elip_plot, = axes.plot(transformed_xys.get_x_data(), transformed_xys.get_y_data(),
                           color=color, **plot_kwargs)
    elems.append(elip_plot)
