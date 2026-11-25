"""
dwa_controller.py

Author: Yuvraj Gupta

Reference:
D. Fox, W. Burgard and S. Thrun,
"The Dynamic Window Approach to Collision Avoidance", IEEE Robotics & Automation Magazine, 1997
"""

from math import atan2, atan
import numpy as np
import numpy.typing as npt


class DwaController:
    """
    Local planner and controller by the Dynamic Window Approach (DWA)
    Every cycle, it samples pairs of (speed, yaw rate) which the vehicle can reach
    within one cycle (the dynamic window), predicts a short trajectory for each pair,
    and picks the pair with the best score of heading to goal, clearance and speed.
    Works as a controller of FourWheelsVehicle.

    Simplifications compared with Fox et al. (1997):
    - Admissible velocities: the paper keeps only speeds from which the vehicle can stop
      before the nearest obstacle (v <= sqrt(2 * dist * braking accel)). Here, a pair is
      dropped if its predicted trajectory comes within ROBOT_RADIUS_M of an obstacle.
    - Objective: the paper normalizes each term over the candidates and smooths the sum.
      Here, each term is divided by its largest possible value.
    - Obstacles are taken directly from ObstacleList, not sensed.
    - The vehicle has no yaw-rate state, so the window around the yaw rate uses the
      previous command.
    """

    def __init__(self, spec, goal_xy: tuple[float, float], obst_list=None):
        """
        Constructor
        spec: VehicleSpecification object
        goal_xy: Goal position (x[m], y[m])
        obst_list: ObstacleList object, can be None
        """

        # vehicle limits
        self.MAX_SPEED_MPS = 15.0 / 3.6
        self.MIN_SPEED_MPS = 0.0
        self.MAX_YAW_RATE_RPS = np.deg2rad(40.0)
        self.MAX_ACCEL_MPS2 = 2.0
        self.MAX_YAW_ACCEL_RPS2 = np.deg2rad(60.0)

        # sampling parameters
        self.SPEED_RESO_MPS = 0.1
        self.YAW_RATE_RESO_RPS = np.deg2rad(2.0)
        self.PREDICT_TIME_SEC = 3.0
        self.PREDICT_DT_SEC = 0.1

        # score weights
        self.K_HEADING = 1.0
        self.K_CLEARANCE = 1.0
        self.K_SPEED = 4.0
        self.ROBOT_RADIUS_M = 2.0  # the vehicle is a circle with this radius around its middle
        self.MAX_CLEARANCE_M = 5.0  # clearance larger than this gives no more score
        self.GOAL_TOLERANCE_M = 1.0

        self.WHEEL_BASE_M = spec.wheel_base_m
        self.goal_xy = np.array(goal_xy, dtype=float)
        self.obst_list = obst_list

        self.target_speed_mps = 0.0
        self.target_yaw_rate_rps = 0.0
        self.target_accel_mps2 = 0.0
        self.target_steer_rad = 0.0

        self.best_trajectory = None
        self.candidate_trajectories = []
        self.dropped_trajectories = []
        self.center_xy = None
        self.goal_reached = False

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

    def _dynamic_window(self, speed_mps: float, yaw_rate_rps: float, time_s: float) -> tuple[float, float, float, float]:
        """
        Private function to calculate the dynamic window
        It is the overlap of the vehicle limits and what is reachable in one cycle
        Return: (min speed, max speed, min yaw rate, max yaw rate)
        """

        min_v = max(self.MIN_SPEED_MPS, speed_mps - self.MAX_ACCEL_MPS2 * time_s)
        max_v = min(self.MAX_SPEED_MPS, speed_mps + self.MAX_ACCEL_MPS2 * time_s)
        min_w = max(-self.MAX_YAW_RATE_RPS, yaw_rate_rps - self.MAX_YAW_ACCEL_RPS2 * time_s)
        max_w = min(self.MAX_YAW_RATE_RPS, yaw_rate_rps + self.MAX_YAW_ACCEL_RPS2 * time_s)
        return min_v, max_v, min_w, max_w

    def _predict_trajectory(self, x: float, y: float, yaw: float,
                            speed_mps: float, yaw_rate_rps: float) -> npt.NDArray[np.float64]:
        """
        Private function to predict a trajectory with constant speed and yaw rate
        Return: Array of (x, y, yaw), shape (N, 3)
        """

        steps = int(self.PREDICT_TIME_SEC / self.PREDICT_DT_SEC)
        traj = np.zeros((steps + 1, 3))
        traj[0] = [x, y, yaw]
        for i in range(steps):
            yaw += yaw_rate_rps * self.PREDICT_DT_SEC
            x += speed_mps * np.cos(yaw) * self.PREDICT_DT_SEC
            y += speed_mps * np.sin(yaw) * self.PREDICT_DT_SEC
            traj[i + 1] = [x, y, yaw]
        return traj

    def _heading_score(self, traj: npt.NDArray[np.float64]) -> float:
        """
        Private function to score how well the trajectory's end points toward the goal
        Return: 1.0 when pointing at the goal, 0.0 when pointing away
        """

        dx, dy = self.goal_xy[0] - traj[-1, 0], self.goal_xy[1] - traj[-1, 1]
        angle_error = atan2(dy, dx) - traj[-1, 2]
        angle_error = abs(atan2(np.sin(angle_error), np.cos(angle_error)))
        return 1.0 - angle_error / np.pi

    def _clearance(self, traj: npt.NDArray[np.float64], obst_points: npt.NDArray[np.float64]) -> float:
        """
        Private function to calculate the smallest distance between the trajectory and obstacles
        Return: Clearance[m] after subtracting the robot radius, negative means collision
        """

        if len(obst_points) == 0: return self.MAX_CLEARANCE_M
        # the trajectory is of the rear axle, so check around the middle of the vehicle
        center_x = traj[:, 0] + self.WHEEL_BASE_M / 2.0 * np.cos(traj[:, 2])
        center_y = traj[:, 1] + self.WHEEL_BASE_M / 2.0 * np.sin(traj[:, 2])
        diff_x = center_x[:, None] - obst_points[None, :, 0]
        diff_y = center_y[:, None] - obst_points[None, :, 1]
        return float(np.min(np.hypot(diff_x, diff_y))) - self.ROBOT_RADIUS_M

    def _search_best_input(self, state, time_s: float) -> None:
        """
        Private function to search the best (speed, yaw rate) in the dynamic window
        state: Vehicle's state object
        time_s: Simulation interval time[sec]
        """

        x, y, yaw = state.get_x_m(), state.get_y_m(), state.get_yaw_rad()
        speed = state.get_speed_mps()
        min_v, max_v, min_w, max_w = self._dynamic_window(speed, self.target_yaw_rate_rps, time_s)
        obst_points = self._obstacle_points()
        self.center_xy = (x + self.WHEEL_BASE_M / 2.0 * np.cos(yaw), y + self.WHEEL_BASE_M / 2.0 * np.sin(yaw))

        candidates, dropped = [], []
        for v in np.arange(min_v, max_v + 1e-6, self.SPEED_RESO_MPS):
            for w in np.arange(min_w, max_w + 1e-6, self.YAW_RATE_RESO_RPS):
                traj = self._predict_trajectory(x, y, yaw, v, w)
                clearance = self._clearance(traj, obst_points)
                if clearance <= 0.0:  # this input would hit an obstacle
                    dropped.append(traj)
                    continue
                candidates.append((v, w, traj, self._heading_score(traj),
                                   min(clearance, self.MAX_CLEARANCE_M), v))

        self.candidate_trajectories = [c[2] for c in candidates]
        self.dropped_trajectories = dropped
        if not candidates:
            # no safe input: brake hard and turn in place as much as possible
            self.target_speed_mps = max(min_v, 0.0)
            self.target_yaw_rate_rps = max_w
            self.best_trajectory = None
            return

        # total score: each term is scaled to about 0..1 by its maximum possible value
        scores = np.array([[c[3], c[4] / self.MAX_CLEARANCE_M, c[5] / self.MAX_SPEED_MPS] for c in candidates])
        total = self.K_HEADING * scores[:, 0] + self.K_CLEARANCE * scores[:, 1] + self.K_SPEED * scores[:, 2]
        best = candidates[int(np.argmax(total))]
        self.target_speed_mps, self.target_yaw_rate_rps, self.best_trajectory = best[0], best[1], best[2]

    def update(self, state, time_s: float) -> None:
        """
        Function to calculate control inputs
        state: Vehicle's state object
        time_s: Simulation interval time[sec]
        """

        dist_to_goal = np.hypot(self.goal_xy[0] - state.get_x_m(), self.goal_xy[1] - state.get_y_m())
        if dist_to_goal < self.GOAL_TOLERANCE_M: self.goal_reached = True

        if self.goal_reached:
            self.target_speed_mps, self.target_yaw_rate_rps = 0.0, 0.0
            self.candidate_trajectories, self.dropped_trajectories, self.best_trajectory = [], [], None
            self.center_xy = None
        else:
            self._search_best_input(state, time_s)

        speed = state.get_speed_mps()
        self.target_accel_mps2 = (self.target_speed_mps - speed) / time_s
        # steering angle which gives the selected yaw rate (only for drawing the tires)
        if abs(speed) > 1e-3:
            self.target_steer_rad = atan(self.WHEEL_BASE_M * self.target_yaw_rate_rps / speed)

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
        Function to draw the goal, the safety radius, the sampled trajectories
        (safe ones in cyan, dropped ones in red) and the selected trajectory
        axes: Axes object of figure
        elems: List of plot objects
        """

        goal_plot, = axes.plot(self.goal_xy[0], self.goal_xy[1], marker='*', ms=15, color='g', label="Goal")
        elems.append(goal_plot)

        if self.center_xy is not None:
            angles = np.linspace(0.0, 2.0 * np.pi, 50)
            radius_plot, = axes.plot(self.center_xy[0] + self.ROBOT_RADIUS_M * np.cos(angles),
                                     self.center_xy[1] + self.ROBOT_RADIUS_M * np.sin(angles),
                                     lw=1.0, ls='--', color='k', label="Safety radius")
            elems.append(radius_plot)

        for traj in self.dropped_trajectories:
            dropped_plot, = axes.plot(traj[:, 0], traj[:, 1], lw=0.7, color='r', alpha=0.5)
            elems.append(dropped_plot)

        for traj in self.candidate_trajectories:
            candidate_plot, = axes.plot(traj[:, 0], traj[:, 1], lw=0.7, color='c', alpha=0.6)
            elems.append(candidate_plot)

        if self.best_trajectory is not None:
            best_plot, = axes.plot(self.best_trajectory[:, 0], self.best_trajectory[:, 1],
                                   lw=2.0, color='m', label="Selected trajectory")
            elems.append(best_plot)
