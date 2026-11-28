"""
scan_matching_odometry.py

Author: Yuvraj Gupta
"""

# import path setting
import sys
from pathlib import Path
from math import atan2
import numpy as np
import numpy.typing as npt

abs_dir_path = str(Path(__file__).absolute().parent)

sys.path.append(abs_dir_path)

# import component modules
from icp_matcher import IcpMatcher


class ScanMatchingOdometry:
    """
    LiDAR odometry by ICP scan matching
    Every cycle, the latest LiDAR scan is aligned to the previous scan by ICP.
    The result is the vehicle's motion between the two scans, and the motions are
    chained to estimate the pose without any wheel speed or GNSS.
    The LiDAR is assumed to be installed at the origin of the vehicle (rear axle center).
    """

    def __init__(self, sensors, init_x_m: float = 0.0, init_y_m: float = 0.0,
                 init_yaw_rad: float = 0.0, color: str = 'b'):
        """
        Constructor
        sensors: Sensors object which has a LiDAR
        init_x_m, init_y_m: Initial position[m]
        init_yaw_rad: Initial yaw angle[rad]
        color: Color of drawing the estimated trajectory
        """

        self.sensors = sensors
        self.matcher = IcpMatcher()
        self.DRAW_COLOR = color

        # estimated pose as a homogeneous transformation matrix, shape (3, 3)
        self.pose = self._to_matrix(np.eye(2), np.array([init_x_m, init_y_m]))
        self.pose[:2, :2] = np.array([[np.cos(init_yaw_rad), -np.sin(init_yaw_rad)],
                                      [np.sin(init_yaw_rad), np.cos(init_yaw_rad)]])

        # previous motion is used as the initial guess (constant velocity assumption)
        self.prev_motion_R = np.eye(2)
        self.prev_motion_t = np.zeros(2)

        self.prev_scan = None
        self.latest_scan = np.zeros((0, 2))
        self.x_history = [init_x_m]
        self.y_history = [init_y_m]

    @staticmethod
    def _to_matrix(R: npt.NDArray[np.float64], t: npt.NDArray[np.float64]) -> npt.NDArray[np.float64]:
        """
        Private function to make a homogeneous transformation matrix
        R: Rotation matrix, shape (2, 2)
        t: Translation vector, shape (2,)
        Return: Matrix, shape (3, 3)
        """

        mat = np.eye(3)
        mat[:2, :2] = R
        mat[:2, 2] = t
        return mat

    def _read_scan(self) -> npt.NDArray[np.float64]:
        """
        Private function to get the latest scan on the sensor coordinate system
        Return: Array of points, shape (N, 2)
        """

        point_cloud = self.sensors.get_point_cloud_from_lidar()
        if not point_cloud: return np.zeros((0, 2))
        return np.array([point.get_point_array()[:, 0] for point in point_cloud])

    def update(self, time_s: float) -> None:
        """
        Function to estimate the motion from the previous scan and update the pose
        time_s: Simulation interval time[sec]
        """

        scan = self._read_scan()
        self.latest_scan = scan

        if self.prev_scan is not None and len(scan) >= 3:
            # current scan -> previous scan gives the current pose seen from the previous pose
            R, t = self.matcher.match(scan, self.prev_scan, self.prev_motion_R, self.prev_motion_t)
            self.pose = self.pose @ self._to_matrix(R, t)
            self.prev_motion_R, self.prev_motion_t = R, t

        self.prev_scan = scan
        self.x_history.append(self.pose[0, 2])
        self.y_history.append(self.pose[1, 2])

    def get_x_m(self) -> float:
        """
        Function to get estimated x[m]
        """

        return self.pose[0, 2]

    def get_y_m(self) -> float:
        """
        Function to get estimated y[m]
        """

        return self.pose[1, 2]

    def get_yaw_rad(self) -> float:
        """
        Function to get estimated yaw angle[rad]
        """

        return atan2(self.pose[1, 0], self.pose[0, 0])

    def draw(self, axes, elems) -> None:
        """
        Function to draw the estimated trajectory and the scan placed at the estimated pose
        axes: Axes object of figure
        elems: List of plot objects
        """

        traj_plot, = axes.plot(self.x_history, self.y_history, lw=2.0, ls='--',
                               color=self.DRAW_COLOR, label="ICP odometry")
        elems.append(traj_plot)

        if len(self.latest_scan):
            global_scan = self.latest_scan @ self.pose[:2, :2].T + self.pose[:2, 2]
            scan_plot, = axes.plot(global_scan[:, 0], global_scan[:, 1], lw=0, marker='.',
                                   ms=3, color=self.DRAW_COLOR)
            elems.append(scan_plot)
