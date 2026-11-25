"""
frenet_optimal_trajectory_planner.py

Author: Yuvraj Gupta

Reference:
M. Werling, J. Ziegler, S. Kammel and S. Thrun,
"Optimal Trajectory Generation for Dynamic Street Scenarios in a Frenet Frame", ICRA 2010
"""

# import path setting
import sys
from pathlib import Path
from math import atan2, cos, sin, tan
import numpy as np
import numpy.typing as npt

abs_dir_path = str(Path(__file__).absolute().parent)
relative_path = "/../../../components/"

sys.path.append(abs_dir_path)
sys.path.append(abs_dir_path + relative_path + "common")

# import component modules
from polynomials import QuinticPolynomial, QuarticPolynomial
from angle_lib import pi_to_pi


class FrenetPath:
    """
    One candidate trajectory, both in the Frenet frame (s, d) and the global frame (x, y)
    """

    def __init__(self):
        self.t = np.array([])  # time[sec]
        self.d = np.array([])  # lateral offset from the reference course[m]
        self.d_d = np.array([])  # lateral velocity[m/s]
        self.d_dd = np.array([])  # lateral acceleration[m/s2]
        self.d_ddd = np.array([])  # lateral jerk[m/s3]
        self.s = np.array([])  # distance along the reference course[m]
        self.s_d = np.array([])  # longitudinal velocity[m/s]
        self.s_dd = np.array([])  # longitudinal acceleration[m/s2]
        self.s_ddd = np.array([])  # longitudinal jerk[m/s3]

        self.x = np.array([])  # global x[m]
        self.y = np.array([])  # global y[m]
        self.curvature = np.array([])  # curvature[1/m]

        self.cost = float("inf")


class FrenetOptimalTrajectoryPlanner:
    """
    Local trajectory planner in the Frenet frame
    It samples many lateral and longitudinal polynomial trajectories along a reference course,
    throws away the ones that are infeasible or hit an obstacle, and picks the cheapest one.
    The picked trajectory is followed with Pure Pursuit, so this class works as a controller
    of FourWheelsVehicle (update, get_target_accel_mps2, get_target_yaw_rate_rps, ...)
    """

    def __init__(self, spec, course, obst_list=None):
        """
        Constructor
        spec: VehicleSpecification object
        course: CubicSplineCourse object used as the reference line (lane center)
        obst_list: ObstacleList object, can be None
        """

        # sampling parameters
        self.MAX_LEFT_OFFSET_M = 4.0  # max lateral offset to the left[m]
        self.MAX_RIGHT_OFFSET_M = 4.0  # max lateral offset to the right[m]
        self.D_OFFSET_STEP_M = 1.0  # lateral sampling step[m]
        self.MIN_HORIZON_SEC = 3.0  # shortest trajectory duration[sec]
        self.MAX_HORIZON_SEC = 5.0  # longest trajectory duration[sec]
        self.HORIZON_STEP_SEC = 1.0  # duration sampling step[sec]
        self.DT_SEC = 0.1  # time step inside one trajectory[sec]
        self.SPEED_SAMPLE_STEP_MPS = 1.0  # end speed sampling step[m/s]
        self.SPEED_SAMPLE_NUM = 1  # end speeds are target +- this number of steps

        # feasibility limits
        self.MAX_SPEED_MPS = 50.0 / 3.6
        self.MAX_ACCEL_MPS2 = 3.0
        self.MAX_CURVATURE = 1.0
        self.COLLISION_RADIUS_M = 2.5  # vehicle is a circle with this radius around the path

        # cost weights
        self.K_JERK = 0.1
        self.K_TIME = 0.1
        self.K_DIFF = 1.0
        self.K_LAT = 1.0
        self.K_LON = 1.0

        # pure pursuit parameters to follow the picked trajectory
        self.LOOK_AHEAD_GAIN = 0.3
        self.LOOK_AHEAD_MIN_M = 3.0
        self.SPEED_GAIN = 1.0

        self.WHEEL_BASE_M = spec.wheel_base_m
        self.course = course
        self.obst_list = obst_list
        self.target_speed_mps = course.target_speed_mps
        self._build_reference_line()

        # previous best path, used to keep lateral velocity/acceleration continuous
        self.best_path = None
        self.candidate_paths = []

        self.target_accel_mps2 = 0.0
        self.target_yaw_rate_rps = 0.0
        self.target_steer_rad = 0.0

    def _build_reference_line(self) -> None:
        """
        Private function to calculate the arc length s of each course point
        """

        self.ref_x = np.array(self.course.x_array)
        self.ref_y = np.array(self.course.y_array)
        self.ref_yaw = np.array(self.course.yaw_array)
        self.ref_curv = np.array(self.course.curvature_array)
        diff = np.hypot(np.diff(self.ref_x), np.diff(self.ref_y))
        self.ref_s = np.concatenate(([0.0], np.cumsum(diff)))

    def _global_to_frenet(self, x_m: float, y_m: float) -> tuple[float, float]:
        """
        Private function to project a global point onto the reference line
        x_m, y_m: Global position[m]
        Return: (s[m], d[m]), d is positive on the left side of the course
        """

        idx = int(np.argmin(np.hypot(self.ref_x - x_m, self.ref_y - y_m)))
        yaw = self.ref_yaw[idx]
        dx, dy = x_m - self.ref_x[idx], y_m - self.ref_y[idx]
        d = -dx * sin(yaw) + dy * cos(yaw)
        return self.ref_s[idx], d

    def _calc_frenet_paths(self, s0: float, s0_d: float, s0_dd: float,
                           d0: float, d0_d: float, d0_dd: float) -> list[FrenetPath]:
        """
        Private function to sample candidate trajectories
        s0, s0_d, s0_dd: Current longitudinal position, velocity, acceleration
        d0, d0_d, d0_dd: Current lateral position, velocity, acceleration
        """

        paths = []
        end_offsets = np.arange(-self.MAX_RIGHT_OFFSET_M, self.MAX_LEFT_OFFSET_M + 1e-6, self.D_OFFSET_STEP_M)
        horizons = np.arange(self.MIN_HORIZON_SEC, self.MAX_HORIZON_SEC + 1e-6, self.HORIZON_STEP_SEC)
        speed_steps = np.arange(-self.SPEED_SAMPLE_NUM, self.SPEED_SAMPLE_NUM + 1)
        end_speeds = self.target_speed_mps + self.SPEED_SAMPLE_STEP_MPS * speed_steps

        for d_end in end_offsets:
            for horizon in horizons:
                t = np.arange(0.0, horizon, self.DT_SEC)

                # lateral motion: start state -> (d_end, 0, 0)
                lat = QuinticPolynomial(d0, d0_d, d0_dd, d_end, 0.0, 0.0, horizon)
                d, d_d = lat.position(t), lat.velocity(t)
                d_dd, d_ddd = lat.acceleration(t), lat.jerk(t)
                lat_jerk_cost = np.sum(d_ddd ** 2)

                for v_end in end_speeds:
                    # longitudinal motion: start state -> (free, v_end, 0)
                    lon = QuarticPolynomial(s0, s0_d, s0_dd, v_end, 0.0, horizon)

                    path = FrenetPath()
                    path.t = t
                    path.d, path.d_d, path.d_dd, path.d_ddd = d, d_d, d_dd, d_ddd
                    path.s = lon.position(t)
                    path.s_d = lon.velocity(t)
                    path.s_dd = lon.acceleration(t)
                    path.s_ddd = lon.jerk(t)

                    # cost = smoothness (jerk) + time + distance from the goal state
                    lon_jerk_cost = np.sum(path.s_ddd ** 2)
                    speed_diff = (self.target_speed_mps - path.s_d[-1]) ** 2
                    cost_lat = self.K_JERK * lat_jerk_cost + self.K_TIME / horizon + self.K_DIFF * d[-1] ** 2
                    cost_lon = self.K_JERK * lon_jerk_cost + self.K_TIME / horizon + self.K_DIFF * speed_diff
                    path.cost = self.K_LAT * cost_lat + self.K_LON * cost_lon

                    paths.append(path)

        return paths

    def _calc_global_paths(self, paths: list[FrenetPath]) -> None:
        """
        Private function to convert each path from the Frenet frame to the global frame
        x = x_ref(s) - d * sin(yaw_ref(s)), y = y_ref(s) + d * cos(yaw_ref(s))
        """

        for path in paths:
            s = np.clip(path.s, self.ref_s[0], self.ref_s[-1])
            ref_x = np.interp(s, self.ref_s, self.ref_x)
            ref_y = np.interp(s, self.ref_s, self.ref_y)
            ref_yaw = np.interp(s, self.ref_s, np.unwrap(self.ref_yaw))
            path.x = ref_x - path.d * np.sin(ref_yaw)
            path.y = ref_y + path.d * np.cos(ref_yaw)

            # curvature from the global path: change of heading per distance
            yaw = np.arctan2(np.diff(path.y), np.diff(path.x))
            ds = np.hypot(np.diff(path.x), np.diff(path.y))
            if len(yaw) > 1:
                curv = np.abs(pi_to_pi(np.diff(yaw))) / np.maximum(ds[:-1], 1e-6)
            else:
                curv = np.zeros(1)
            path.curvature = curv

    def _obstacle_points(self) -> npt.NDArray[np.float64]:
        """
        Private function to sample points on each obstacle's outline
        Return: Array of points, shape (N, 2)
        """

        points = []
        if self.obst_list is None: return np.zeros((0, 2))
        for obst in self.obst_list.get_list():
            vx, vy = obst.vertex_xy()
            for i in range(len(vx) - 1):
                n = max(int(np.hypot(vx[i + 1] - vx[i], vy[i + 1] - vy[i]) / 0.5), 1)
                for r in np.linspace(0.0, 1.0, n, endpoint=False):
                    points.append([vx[i] + r * (vx[i + 1] - vx[i]), vy[i] + r * (vy[i + 1] - vy[i])])
        return np.array(points) if points else np.zeros((0, 2))

    def _is_collision_free(self, path: FrenetPath, obst_points: npt.NDArray[np.float64]) -> bool:
        """
        Private function to check the distance between the path and all obstacle points
        obst_points: Array of obstacle points, shape (N, 2)
        """

        if len(obst_points) == 0: return True
        diff_x = path.x[:, None] - obst_points[None, :, 0]
        diff_y = path.y[:, None] - obst_points[None, :, 1]
        return bool(np.min(np.hypot(diff_x, diff_y)) > self.COLLISION_RADIUS_M)

    def _check_paths(self, paths: list[FrenetPath]) -> list[FrenetPath]:
        """
        Private function to keep only feasible and collision free paths
        """

        obst_points = self._obstacle_points()
        valid = []
        for path in paths:
            if np.max(path.s_d) > self.MAX_SPEED_MPS: continue
            if np.max(np.abs(path.s_dd)) > self.MAX_ACCEL_MPS2: continue
            if np.max(path.curvature) > self.MAX_CURVATURE: continue
            if not self._is_collision_free(path, obst_points): continue
            valid.append(path)
        return valid

    def _plan(self, state) -> None:
        """
        Private function to plan the best trajectory from the current state
        state: Vehicle's state object
        """

        s0, d0 = self._global_to_frenet(state.get_x_m(), state.get_y_m())

        # velocity and acceleration are taken from the previous plan,
        # so the new trajectory starts smoothly from the old one
        if self.best_path is not None and len(self.best_path.t) > 1:
            d0_d, d0_dd = self.best_path.d_d[1], self.best_path.d_dd[1]
            s0_dd = self.best_path.s_dd[1]
        else:
            d0_d, d0_dd, s0_dd = 0.0, 0.0, 0.0
        s0_d = state.get_speed_mps()

        paths = self._calc_frenet_paths(s0, s0_d, s0_dd, d0, d0_d, d0_dd)
        self._calc_global_paths(paths)
        valid = self._check_paths(paths)

        self.candidate_paths = paths
        if valid:
            self.best_path = min(valid, key=lambda p: p.cost)
        # if no path is valid, keep following the previous best path

    def _follow_best_path(self, state) -> None:
        """
        Private function to follow the best path by Pure Pursuit
        state: Vehicle's state object
        """

        if self.best_path is None: return

        x, y, yaw = state.get_x_m(), state.get_y_m(), state.get_yaw_rad()
        speed = state.get_speed_mps()

        # look ahead point on the best path
        look_ahead_m = self.LOOK_AHEAD_GAIN * speed + self.LOOK_AHEAD_MIN_M
        dist = np.hypot(self.best_path.x - x, self.best_path.y - y)
        far_idx = np.where(dist >= look_ahead_m)[0]
        idx = far_idx[0] if len(far_idx) else len(dist) - 1

        alpha = pi_to_pi(atan2(self.best_path.y[idx] - y, self.best_path.x[idx] - x) - yaw)
        self.target_steer_rad = atan2(2.0 * self.WHEEL_BASE_M * sin(alpha), look_ahead_m)
        self.target_yaw_rate_rps = speed * tan(self.target_steer_rad) / self.WHEEL_BASE_M

        target_speed = self.best_path.s_d[min(1, len(self.best_path.s_d) - 1)]
        self.target_accel_mps2 = self.SPEED_GAIN * (target_speed - speed)

    def update(self, state, time_s: float) -> None:
        """
        Function to replan and calculate control inputs
        state: Vehicle's state object
        time_s: Simulation interval time[sec]
        """

        self._plan(state)
        self._follow_best_path(state)

    def get_target_accel_mps2(self) -> float:
        """
        Function to get acceleration input[m/s2]
        """

        return self.target_accel_mps2

    def get_target_yaw_rate_rps(self) -> float:
        """
        Function to get yaw rate input[rad/s]
        """

        return self.target_yaw_rate_rps

    def get_target_steer_rad(self) -> float:
        """
        Function to get steering angle input[rad]
        """

        return self.target_steer_rad

    def draw(self, axes, elems) -> None:
        """
        Function to draw the candidate paths and the best path
        axes: Axes object of figure
        elems: List of plot objects
        """

        for path in self.candidate_paths:
            candidate_plot, = axes.plot(path.x, path.y, lw=0.5, color='c', alpha=0.4)
            elems.append(candidate_plot)

        if self.best_path is not None:
            best_plot, = axes.plot(self.best_path.x, self.best_path.y, lw=2.0, color='m', label="Best path")
            elems.append(best_plot)
