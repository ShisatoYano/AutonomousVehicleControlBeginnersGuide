"""
landmark.py

Static point landmark for SLAM (mirrors obstacle.Obstacle: data + draw).
Noise-free range–bearing prediction lives here; EKF Jacobians stay in observation_model.
"""

import numpy as np
from math import atan2, sqrt


class Landmark:
    """
    A single 2D point landmark at fixed world coordinates.
    """

    def __init__(self, x_m, y_m, color="k", marker_size=8):
        """
        x_m, y_m: position in global frame [m]
        color: matplotlib color for draw()
        marker_size: marker size for draw()
        """
        self.x_m = float(x_m)
        self.y_m = float(y_m)
        self.DRAW_COLOR = color
        self.marker_size = marker_size

    def get_x_m(self):
        return self.x_m

    def get_y_m(self):
        return self.y_m

    def position_xy(self):
        """Returns (x_m, y_m) tuple."""
        return self.x_m, self.y_m

    def predicted_range_bearing(self, robot_state):
        """
        Noise-free predicted [range, bearing] from robot pose to this landmark.
        robot_state: (3, 1) or (4, 1); only x, y, yaw are used.
        Returns: (2, 1) ndarray
        """
        return Landmark.predicted_range_bearing_at(robot_state, self.x_m, self.y_m)

    @staticmethod
    def predicted_range_bearing_at(robot_state, lx, ly):
        """
        Same geometry as legacy observe_landmark; for EKF when reading lx, ly from state.
        robot_state: (3, 1) [x, y, yaw]
        lx, ly: landmark position [m]
        """
        x = robot_state[0, 0]
        y = robot_state[1, 0]
        yaw = robot_state[2, 0]
        dx = lx - x
        dy = ly - y
        r = sqrt(dx * dx + dy * dy)
        if r < 1e-9:
            return np.array([[0.0], [0.0]])
        global_bearing = atan2(dy, dx)
        bearing = (global_bearing - yaw + np.pi) % (2 * np.pi) - np.pi
        return np.array([[r], [bearing]])

    def draw(self, axes, elems, label=None):
        """
        Draw landmark as a point marker (similar role to Obstacle.draw).
        """
        plot_kwargs = {
            "markersize": self.marker_size,
            "color": self.DRAW_COLOR,
            "linestyle": "None",
            "marker": "o",
        }
        if label is not None:
            plot_kwargs["label"] = label
        p, = axes.plot([self.x_m], [self.y_m], **plot_kwargs)
        elems.append(p)
