"""
mppi_controller.py

Model Predictive Path Integral (MPPI) controller for path tracking.
Follows the standard MPPI formulation: warm-started control sequence, additive
noise sampling, stage and terminal costs, information-theoretic weighting,
and optional moving-average smoothing. Uses (steer, accel) as control input;
converts to (accel, yaw_rate) for the existing State.motion_model. Visualizes
optimal and optionally all sampled trajectories.
"""

import math
import sys
from pathlib import Path
from math import atan2, cos, sin, tan
import numpy as np

abs_dir_path = str(Path(__file__).absolute().parent)
relative_path = "/../../../components/"
sys.path.append(abs_dir_path + relative_path + "state")
sys.path.append(abs_dir_path + relative_path + "course/cubic_spline_course")

from state import State


class _StateView:
    """Minimal state-like object for course methods (x, y, yaw, speed)."""

    def __init__(self, x_m, y_m, yaw_rad, speed_mps):
        self.x_m = x_m
        self.y_m = y_m
        self.yaw_rad = yaw_rad
        self.speed_mps = speed_mps

    def get_x_m(self):
        return self.x_m

    def get_y_m(self):
        return self.y_m

    def get_yaw_rad(self):
        return self.yaw_rad

    def get_speed_mps(self):
        return self.speed_mps


class MppiController:
    """
    MPPI path-tracking controller aligned with standard formulation:
    warm start (u_prev), exploitation/exploration sampling, stage + terminal cost,
    control cost term (param_gamma * u.T @ inv(Sigma) @ v), and optional smoothing.
    Control input is (steer_rad, accel_mps2); outputs match vehicle interface
    (accel, yaw_rate, steer for draw).
    """

    def __init__(
        self,
        spec,
        course=None,
        color="g",
        delta_t=0.05,
        horizon_step_T=20,
        number_of_samples_K=256,
        param_exploration=0.0,
        param_lambda=50.0,
        param_alpha=1.0,
        sigma_steer=0.1,
        sigma_accel=0.5,
        max_steer_abs=0.523,
        max_accel_abs=2.0,
        stage_cost_weight=None,
        terminal_cost_weight=None,
        moving_average_window=0,
        visualize_optimal_traj=True,
        visualize_sampled_trajs=True,
    ):
        """
        spec: Vehicle specification (wheel_base_m used for motion model).
        course: Reference path with search_nearest_point_index, point_x_m, point_y_m,
                point_yaw_rad, point_speed_mps.
        color: Color for optimal trajectory.
        delta_t: Time step for rollout [s].
        horizon_step_T: Prediction horizon (number of steps).
        number_of_samples_K: Number of sample trajectories.
        param_exploration: Fraction of samples that use pure exploration (no u_prev).
        param_lambda: MPPI temperature (lambda).
        param_alpha: MPPI alpha; param_gamma = param_lambda * (1 - param_alpha).
        sigma_steer, sigma_accel: Std for steer and accel noise (used to build Sigma).
        max_steer_abs: Maximum steering angle [rad].
        max_accel_abs: Maximum acceleration [m/s^2].
        stage_cost_weight: [x, y, yaw, v] stage cost weights (default [50, 50, 1, 20]).
        terminal_cost_weight: [x, y, yaw, v] terminal cost weights (default same as stage).
        moving_average_window: Window for smoothing w_epsilon (0 = disable).
        visualize_optimal_traj: If True, draw optimal trajectory.
        visualize_sampled_trajs: If True, draw all sampled trajectories.
        """
        self.WHEEL_BASE_M = spec.wheel_base_m
        self.course = course
        self.DRAW_COLOR = color
        self.delta_t = delta_t
        self.T = horizon_step_T
        self.K = number_of_samples_K
        self.param_exploration = max(0.0, min(1.0, param_exploration))
        self.param_lambda = max(1e-6, param_lambda)
        self.param_alpha = param_alpha
        self.param_gamma = self.param_lambda * (1.0 - self.param_alpha)
        self.max_steer_abs = max_steer_abs
        self.max_accel_abs = max_accel_abs
        self.moving_average_window = max(0, moving_average_window)
        self.visualize_optimal_traj = visualize_optimal_traj
        self.visualize_sampled_trajs = visualize_sampled_trajs

        self.Sigma = np.array([[sigma_steer**2, 0.0], [0.0, sigma_accel**2]])
        self.stage_cost_weight = np.asarray(
            stage_cost_weight
            if stage_cost_weight is not None
            else [50.0, 50.0, 1.0, 20.0]
        )
        self.terminal_cost_weight = np.asarray(
            terminal_cost_weight
            if terminal_cost_weight is not None
            else self.stage_cost_weight.copy()
        )

        self.u_prev = np.zeros((self.T, 2))  # (steer, accel) per step
        self.prev_waypoints_idx = 0

        self.target_accel_mps2 = 0.0
        self.target_speed_mps = 0.0
        self.target_steer_rad = 0.0
        self.target_yaw_rate_rps = 0.0
        self.optimal_trajectory = None  # (x_list, y_list)
        self.sampled_trajectories = []  # list of (x_list, y_list)
        self.weights = []

    def _get_nearest_waypoint(self, x, y, update_prev_idx=False):
        """Return (ref_x, ref_y, ref_yaw, ref_v) from course at nearest point to (x, y)."""
        if not self.course:
            return 0.0, 0.0, 0.0, 0.0
        view = _StateView(x, y, 0.0, 0.0)
        nearest_idx = self.course.search_nearest_point_index(view)
        ref_x = self.course.point_x_m(nearest_idx)
        ref_y = self.course.point_y_m(nearest_idx)
        ref_yaw = self.course.point_yaw_rad(nearest_idx)
        ref_v = self.course.point_speed_mps(nearest_idx)
        if update_prev_idx:
            self.prev_waypoints_idx = nearest_idx
        return ref_x, ref_y, ref_yaw, ref_v

    def _g(self, v):
        """Clamp control (steer, accel) to limits."""
        steer = np.clip(v[0], -self.max_steer_abs, self.max_steer_abs)
        accel = np.clip(v[1], -self.max_accel_abs, self.max_accel_abs)
        return np.array([steer, accel])

    def _F(self, x_t, v_t):
        """Next state from (x,y,yaw,v) and control (steer, accel). Uses State.motion_model with (accel, yaw_rate)."""
        x, y, yaw, v = x_t[0, 0], x_t[1, 0], x_t[2, 0], x_t[3, 0]
        steer, accel = float(v_t[0]), float(v_t[1])
        if abs(v) < 1e-9:
            yaw_rate = 0.0
        else:
            yaw_rate = v / self.WHEEL_BASE_M * tan(steer)
        state_vec = np.array([[x], [y], [yaw], [v]])
        input_vec = np.array([[accel], [yaw_rate]])
        return State.motion_model(state_vec, input_vec, self.delta_t)

    def _c(self, x_t):
        """Stage cost: weighted squared error to reference + control cost term (filled in by caller with u_prev, v)."""
        x, y, yaw, v = (
            float(x_t[0, 0]),
            float(x_t[1, 0]),
            float(x_t[2, 0]),
            float(x_t[3, 0]),
        )
        yaw = (yaw + 2.0 * np.pi) % (2.0 * np.pi)
        ref_x, ref_y, ref_yaw, ref_v = self._get_nearest_waypoint(x, y)
        ref_yaw = (ref_yaw + 2.0 * np.pi) % (2.0 * np.pi)
        yaw_diff = np.arctan2(np.sin(yaw - ref_yaw), np.cos(yaw - ref_yaw))
        cost = (
            self.stage_cost_weight[0] * (x - ref_x) ** 2
            + self.stage_cost_weight[1] * (y - ref_y) ** 2
            + self.stage_cost_weight[2] * (yaw_diff**2)
            + self.stage_cost_weight[3] * (v - ref_v) ** 2
        )
        return cost

    def _phi(self, x_T):
        """Terminal cost."""
        x, y, yaw, v = (
            float(x_T[0, 0]),
            float(x_T[1, 0]),
            float(x_T[2, 0]),
            float(x_T[3, 0]),
        )
        yaw = (yaw + 2.0 * np.pi) % (2.0 * np.pi)
        ref_x, ref_y, ref_yaw, ref_v = self._get_nearest_waypoint(x, y)
        ref_yaw = (ref_yaw + 2.0 * np.pi) % (2.0 * np.pi)
        yaw_diff = np.arctan2(np.sin(yaw - ref_yaw), np.cos(yaw - ref_yaw))
        cost = (
            self.terminal_cost_weight[0] * (x - ref_x) ** 2
            + self.terminal_cost_weight[1] * (y - ref_y) ** 2
            + self.terminal_cost_weight[2] * (yaw_diff**2)
            + self.terminal_cost_weight[3] * (v - ref_v) ** 2
        )
        return cost

    def _calc_epsilon(self):
        """Sample epsilon (K, T, 2) from N(0, Sigma)."""
        mu = np.zeros(2)
        epsilon = np.random.multivariate_normal(mu, self.Sigma, (self.K, self.T))
        return epsilon

    def _compute_weights(self, S):
        """Information-theoretic weights: rho = min(S), w[k] = (1/eta) * exp(-(S[k]-rho)/lambda)."""
        rho = S.min()
        eta = np.sum(np.exp((-1.0 / self.param_lambda) * (S - rho)))
        w = (1.0 / eta) * np.exp((-1.0 / self.param_lambda) * (S - rho))
        return w

    def _moving_average_filter(self, xx, window_size):
        """Smooth each column of xx (T, 2) with moving average. Same logic as reference."""
        if window_size < 2:
            return xx
        b = np.ones(window_size) / window_size
        xx_mean = np.zeros_like(xx)
        for d in range(xx.shape[1]):
            xx_mean[:, d] = np.convolve(xx[:, d], b, mode="same")
            n_conv = math.ceil(window_size / 2)
            xx_mean[0, d] *= window_size / n_conv
            for i in range(1, n_conv):
                xx_mean[i, d] *= window_size / (i + n_conv)
                xx_mean[-i, d] *= window_size / (i + n_conv - (window_size % 2))
        return xx_mean

    def update(self, state, time_s):
        """
        Run one MPPI step: warm start, sample, rollout, cost, weight, update u_prev,
        set target from first control. Store optimal and sampled trajectories for draw().
        """
        if not self.course:
            self.target_accel_mps2 = 0.0
            self.target_yaw_rate_rps = 0.0
            self.target_steer_rad = 0.0
            self.target_speed_mps = state.get_speed_mps()
            self.optimal_trajectory = None
            self.sampled_trajectories = []
            self.weights = []
            return

        x0 = np.array(
            [
                [state.get_x_m()],
                [state.get_y_m()],
                [state.get_yaw_rad()],
                [state.get_speed_mps()],
            ]
        )
        self._get_nearest_waypoint(x0[0, 0], x0[1, 0], update_prev_idx=True)

        u = self.u_prev.copy()
        epsilon = self._calc_epsilon()
        v = np.zeros((self.K, self.T, 2))
        n_exploit = int((1.0 - self.param_exploration) * self.K)
        for k in range(self.K):
            for t in range(self.T):
                if k < n_exploit:
                    v[k, t] = u[t] + epsilon[k, t]
                else:
                    v[k, t] = epsilon[k, t]
                v[k, t] = self._g(v[k, t])

        S = np.zeros(self.K)
        Sigma_inv = np.linalg.inv(self.Sigma)
        for k in range(self.K):
            x = x0.copy()
            for t in range(self.T):
                u_t = u[t]
                v_t = v[k, t]
                S[k] += self._c(x) + self.param_gamma * (u_t.T @ Sigma_inv @ v_t)
                x = self._F(x, v_t)
            S[k] += self._phi(x)

        w = self._compute_weights(S)
        self.weights = w.tolist()

        w_epsilon = np.zeros((self.T, 2))
        for t in range(self.T):
            for k in range(self.K):
                w_epsilon[t] += w[k] * epsilon[k, t]
        if self.moving_average_window >= 2:
            w_epsilon = self._moving_average_filter(
                w_epsilon, self.moving_average_window
            )
        u = u + w_epsilon
        u = np.clip(
            u,
            [-self.max_steer_abs, -self.max_accel_abs],
            [self.max_steer_abs, self.max_accel_abs],
        )

        steer0 = float(u[0, 0])
        accel0 = float(u[0, 1])
        self.target_steer_rad = steer0
        self.target_accel_mps2 = accel0
        v0 = state.get_speed_mps()
        if abs(v0) < 1e-9:
            self.target_yaw_rate_rps = 0.0
        else:
            self.target_yaw_rate_rps = v0 / self.WHEEL_BASE_M * tan(steer0)
        self.target_speed_mps = v0

        if self.visualize_optimal_traj:
            x = x0.copy()
            x_list = [float(x[0, 0])]
            y_list = [float(x[1, 0])]
            for t in range(self.T):
                x = self._F(x, u[t])
                x_list.append(float(x[0, 0]))
                y_list.append(float(x[1, 0]))
            self.optimal_trajectory = (x_list, y_list)
        else:
            self.optimal_trajectory = None

        self.sampled_trajectories = []
        if self.visualize_sampled_trajs:
            for k in range(self.K):
                x = x0.copy()
                x_list = [float(x[0, 0])]
                y_list = [float(x[1, 0])]
                for t in range(self.T):
                    x = self._F(x, v[k, t])
                    x_list.append(float(x[0, 0]))
                    y_list.append(float(x[1, 0]))
                self.sampled_trajectories.append((x_list, y_list))

        self.u_prev[:-1] = u[1:]
        self.u_prev[-1] = u[-1]

    def get_target_accel_mps2(self):
        return self.target_accel_mps2

    def get_target_steer_rad(self):
        return self.target_steer_rad

    def get_target_yaw_rate_rps(self):
        return self.target_yaw_rate_rps

    def draw(self, axes, elems):
        """Draw sampled trajectories (if enabled) and optimal trajectory (if enabled)."""
        if self.visualize_sampled_trajs and self.sampled_trajectories:
            for (x_list, y_list), w in zip(self.sampled_trajectories, self.weights):
                alpha = 0.06 + 0.12 * min(1.0, float(w) * self.K)
                (line,) = axes.plot(x_list, y_list, "b-", linewidth=0.35, alpha=alpha)
                elems.append(line)
        if self.visualize_optimal_traj and self.optimal_trajectory:
            x_list, y_list = self.optimal_trajectory
            (line,) = axes.plot(
                x_list,
                y_list,
                color=self.DRAW_COLOR,
                linewidth=2.0,
                alpha=0.9,
                label="MPPI trajectory",
            )
            elems.append(line)
