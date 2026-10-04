"""
Unit test of plot_lib's covariance ellipse

Author: Dipak Chaudhari
"""

import numpy as np
import sys
from pathlib import Path

sys.path.append(str(Path(__file__).absolute().parent) + "/../src/components/common")
sys.path.append(str(Path(__file__).absolute().parent) + "/../src/components/localization/kalman_filter")
from plot_lib import draw_covariance_ellipse
from extended_kalman_filter_localizer import ExtendedKalmanFilterLocalizer


class MockAxes:
    """
    Axes stand-in that records the data and keyword arguments of each plotted line
    """

    def __init__(self):
        self.lines = []

    def plot(self, x_data, y_data, *args, **kwargs):
        self.lines.append((np.array(x_data), np.array(y_data), kwargs))
        return (None,)


def rotated_cov(var_major, var_minor, angle_rad):
    rot = np.array([[np.cos(angle_rad), -np.sin(angle_rad)],
                    [np.sin(angle_rad), np.cos(angle_rad)]])
    return rot @ np.diag([var_major, var_minor]) @ rot.T


def squared_mahalanobis(xs, ys, x, y, cov):
    d = np.vstack([xs - x, ys - y])
    return np.einsum("ij,ij->j", d, np.linalg.solve(cov, d))


def test_ellipse_points_are_at_mahalanobis_distance_sqrt_3():
    cov = rotated_cov(4.0, 1.0, np.deg2rad(30.0))
    axes, elems = MockAxes(), []
    draw_covariance_ellipse(axes, elems, 2.0, -1.0, cov)

    xs, ys, kwargs = axes.lines[0]
    assert np.allclose(squared_mahalanobis(xs, ys, 2.0, -1.0, cov), 3.0)
    assert kwargs["color"] == 'r'
    assert len(elems) == 1


def test_plot_kwargs_are_passed_through():
    axes = MockAxes()
    draw_covariance_ellipse(axes, [], 0.0, 0.0, np.eye(2), color='b', linewidth=1.5)

    _, _, kwargs = axes.lines[0]
    assert kwargs == {"color": 'b', "linewidth": 1.5}


def test_degenerate_covariance_is_drawn_as_a_line():
    # rank 1: all variance along x
    axes = MockAxes()
    draw_covariance_ellipse(axes, [], 0.0, 0.0, np.array([[2.0, 0.0], [0.0, 0.0]]))

    xs, ys, _ = axes.lines[0]
    assert np.all(np.isfinite(xs)) and np.all(np.isfinite(ys))
    assert np.allclose(ys, 0.0)
    assert np.isclose(np.max(np.abs(xs)), np.sqrt(6.0), atol=1e-2)


def test_ekf_draws_the_xy_block_of_its_covariance():
    # The yaw and speed variances, and the x-yaw correlation, must not leak into
    # the position ellipse. With eig of the whole 4x4 matrix, these points were at
    # squared Mahalanobis distances from 6.6 to 132 instead of 3.
    ekf = ExtendedKalmanFilterLocalizer()
    cov = np.zeros((4, 4))
    cov[:2, :2] = rotated_cov(4.0, 1.0, np.deg2rad(30.0))
    cov[2, 2], cov[3, 3] = 100.0, 9.0
    cov[0, 2] = cov[2, 0] = 5.0
    ekf.cov_mat = cov
    axes = MockAxes()
    ekf.draw(axes, [], np.array([[1.0], [2.0], [0.0]]))

    xs, ys, _ = axes.lines[0]
    assert np.allclose(squared_mahalanobis(xs, ys, 1.0, 2.0, ekf.cov_mat[:2, :2]), 3.0)
