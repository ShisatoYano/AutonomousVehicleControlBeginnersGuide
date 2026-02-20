"""
ekf_slam.py

EKF-SLAM (Extended Kalman Filter based Simultaneous Localization and Mapping).
Real-time landmark-based SLAM in 2D with range-bearing sensor.
Demonstrates loop closure via covariance convergence when re-observing landmarks.
"""

import sys
import numpy as np
from pathlib import Path
from math import cos, sin, sqrt, atan2, pi

abs_dir_path = str(Path(__file__).absolute().parent)
relative_path = "/../../../components/"
sys.path.append(abs_dir_path + relative_path + "visualization")
sys.path.append(abs_dir_path + relative_path + "state")
sys.path.append(abs_dir_path + relative_path + "array")

from global_xy_visualizer import GlobalXYVisualizer
from min_max import MinMax
from time_parameters import TimeParameters
from xy_array import XYArray

# Local EKF-SLAM modules
from motion_model import predict_robot_state, jacobian_F, jacobian_G
from observation_model import (
    observe_landmark,
    build_H_matrix,
    simulate_observation,
)
from data_association import nearest_neighbor_association
from landmark_manager import augment_state

show_plot = True

# Default parameters
SIGMA_R = 0.2
SIGMA_PHI = 0.1
SIGMA_V = 0.1
SIGMA_OMEGA = 0.05
GATE_THRESHOLD = 3.0
MAX_RANGE = 8.0
NUM_LANDMARKS = 12
SIMULATION_SPAN_SEC = 30.0
INTERVAL_SEC = 0.2


class EKFSLAMSimulation:
    """
    Single object for the visualizer: update(dt) and draw(axes, elems).
    Holds true robot, true landmarks, EKF state (mu, Sigma), and draws everything.
    """

    def __init__(self, true_landmarks_xy, sigma_r=SIGMA_R, sigma_phi=SIGMA_PHI,
                 sigma_v=SIGMA_V, sigma_omega=SIGMA_OMEGA, gate_threshold=GATE_THRESHOLD,
                 max_range=MAX_RANGE, radius=10.0, speed=0.5):
        self.true_landmarks = true_landmarks_xy  # list of (x, y)
        self.sigma_r = sigma_r
        self.sigma_phi = sigma_phi
        self.sigma_v = sigma_v
        self.sigma_omega = sigma_omega
        self.gate_threshold = gate_threshold
        self.max_range = max_range
        self.radius = radius
        self.speed = speed
        self.R_obs = np.diag([sigma_r ** 2, sigma_phi ** 2])
        self.Q_input = np.diag([sigma_v ** 2, sigma_omega ** 2])
        # True robot state [x, y, yaw]
        self.true_robot = np.array([[radius], [0.0], [-pi / 2]])
        self.true_trajectory_x = [float(self.true_robot[0, 0])]
        self.true_trajectory_y = [float(self.true_robot[1, 0])]
        # EKF state: mu = [x, y, yaw, l0_x, l0_y, ...], initially only robot
        self.mu = self.true_robot.copy()
        self.Sigma = np.eye(3) * 0.1
        self.landmark_ids = []  # order of landmarks in state (same as appearance)
        self.est_trajectory_x = [float(self.mu[0, 0])]
        self.est_trajectory_y = [float(self.mu[1, 0])]

    def _control(self, t):
        """Circular path: v constant, omega constant."""
        return np.array([[self.speed], [self.speed / self.radius]])

    def update(self, dt):
        # True motion with process noise
        u = self._control(0)
        u_noisy = u + np.sqrt(self.Q_input) @ np.random.randn(2, 1)
        self.true_robot = predict_robot_state(self.true_robot, u_noisy, dt)
        self.true_trajectory_x.append(float(self.true_robot[0, 0]))
        self.true_trajectory_y.append(float(self.true_robot[1, 0]))

        # EKF prediction
        n = self.mu.shape[0]
        robot_state = self.mu[0:3]
        u_est = u
        robot_pred = predict_robot_state(robot_state, u_est, dt)
        F = jacobian_F(robot_state, u_est, dt)
        G = jacobian_G(robot_state, u_est, dt)
        Sigma_rr = self.Sigma[0:3, 0:3]
        Sigma_rr_new = F @ Sigma_rr @ F.T + G @ self.Q_input @ G.T
        self.mu[0:3] = robot_pred
        if n == 3:
            self.Sigma[0:3, 0:3] = Sigma_rr_new
        else:
            Sigma_rl = self.Sigma[0:3, 3:]
            self.Sigma[0:3, 0:3] = Sigma_rr_new
            self.Sigma[0:3, 3:] = F @ Sigma_rl
            self.Sigma[3:, 0:3] = Sigma_rl.T @ F.T

        # Observations: for each true landmark in range, simulate measurement
        observations = []
        observed_landmark_indices = []
        for idx, (lx, ly) in enumerate(self.true_landmarks):
            z_pred = observe_landmark(self.true_robot, lx, ly)
            if z_pred[0, 0] <= self.max_range:
                z = simulate_observation(
                    self.true_robot, lx, ly, self.sigma_r, self.sigma_phi
                )
                observations.append(z)
                observed_landmark_indices.append(idx)

        if not observations:
            self.est_trajectory_x.append(float(self.mu[0, 0]))
            self.est_trajectory_y.append(float(self.mu[1, 0]))
            return

        # Data association: match observations to known landmarks or new
        pred_obs_list = []
        S_list = []
        for j in range(len(self.landmark_ids)):
            lx = self.mu[3 + 2 * j, 0]
            ly = self.mu[3 + 2 * j + 1, 0]
            z_pred = observe_landmark(self.mu[0:3], lx, ly)
            pred_obs_list.append(z_pred)
            H = build_H_matrix(self.mu[0:3], lx, ly, j, n)
            S = H @ self.Sigma @ H.T + self.R_obs
            S_list.append(S)
        if not pred_obs_list:
            matches = [(o, -1) for o in range(len(observations))]
        else:
            matches = nearest_neighbor_association(
                observations, pred_obs_list, S_list,
                list(range(len(self.landmark_ids))), self.gate_threshold
            )

        # Process matches: updates for known landmarks, augment for new
        for (obs_idx, land_id) in matches:
            z = observations[obs_idx]
            if land_id >= 0:
                # EKF update for this landmark
                j = land_id
                lx = self.mu[3 + 2 * j, 0]
                ly = self.mu[3 + 2 * j + 1, 0]
                z_pred = observe_landmark(self.mu[0:3], lx, ly)
                H = build_H_matrix(self.mu[0:3], lx, ly, j, n)
                S = H @ self.Sigma @ H.T + self.R_obs
                K = self.Sigma @ H.T @ np.linalg.inv(S)
                innov = z - z_pred
                innov[1, 0] = (innov[1, 0] + pi) % (2 * pi) - pi
                self.mu = self.mu + K @ innov
                self.Sigma = (np.eye(n) - K @ H) @ self.Sigma
            else:
                # New landmark: use first observation that was assigned new
                self.mu, self.Sigma = augment_state(
                    self.mu, self.Sigma, z, self.mu[0:3], self.R_obs
                )
                self.landmark_ids.append(observed_landmark_indices[obs_idx])
                n = self.mu.shape[0]

        self.est_trajectory_x.append(float(self.mu[0, 0]))
        self.est_trajectory_y.append(float(self.mu[1, 0]))

    def draw(self, axes, elems):
        # True landmarks
        lx_true = [p[0] for p in self.true_landmarks]
        ly_true = [p[1] for p in self.true_landmarks]
        p1, = axes.plot(lx_true, ly_true, "ko", markersize=8, label="True landmarks")
        elems.append(p1)
        # True trajectory
        p2, = axes.plot(
            self.true_trajectory_x, self.true_trajectory_y,
            "b-", linewidth=1, alpha=0.7, label="True path"
        )
        elems.append(p2)
        # Estimated trajectory
        p3, = axes.plot(
            self.est_trajectory_x, self.est_trajectory_y,
            "r-", linewidth=1, alpha=0.7, label="Est. path"
        )
        elems.append(p3)
        # Estimated robot position
        p4, = axes.plot(
            self.mu[0, 0], self.mu[1, 0], "r^", markersize=10, label="Est. robot"
        )
        elems.append(p4)
        # True robot position
        p5, = axes.plot(
            self.true_robot[0, 0], self.true_robot[1, 0],
            "bs", markersize=8, label="True robot"
        )
        elems.append(p5)
        # Estimated landmarks
        n = self.mu.shape[0]
        if n > 3:
            lx_est = [self.mu[3 + 2 * j, 0] for j in range((n - 3) // 2)]
            ly_est = [self.mu[3 + 2 * j + 1, 0] for j in range((n - 3) // 2)]
            p6, = axes.plot(lx_est, ly_est, "rx", markersize=6, label="Est. landmarks")
            elems.append(p6)
        # Robot covariance ellipse (3-sigma)
        Sigma_rr = self.Sigma[0:3, 0:3]
        Sigma_xy = Sigma_rr[0:2, 0:2]
        try:
            eig_val, eig_vec = np.linalg.eig(Sigma_xy)
            if eig_val[0] >= eig_val[1]:
                big_idx, small_idx = 0, 1
            else:
                big_idx, small_idx = 1, 0
            a = sqrt(3.0 * max(0, eig_val[big_idx]))
            b = sqrt(3.0 * max(0, eig_val[small_idx]))
            angle = atan2(eig_vec[1, big_idx], eig_vec[0, big_idx])
            t = np.arange(0, 2 * pi + 0.1, 0.1)
            xs = a * np.cos(t)
            ys = b * np.sin(t)
            xys = np.array([xs, ys])
            xys_array = XYArray(xys)
            pose = [self.mu[0, 0], self.mu[1, 0], angle]
            trans = xys_array.homogeneous_transformation(pose[0], pose[1], pose[2])
            p7, = axes.plot(trans.get_x_data(), trans.get_y_data(), "g-", linewidth=0.8)
            elems.append(p7)
        except (np.linalg.LinAlgError, ValueError):
            pass


def main():
    np.random.seed(42)
    # Landmarks around a circle (loop-closure scenario)
    num_landmarks = NUM_LANDMARKS
    radius_lm = 10.0
    true_landmarks = [
        (
            radius_lm * np.cos(2 * pi * i / num_landmarks),
            radius_lm * np.sin(2 * pi * i / num_landmarks),
        )
        for i in range(num_landmarks)
    ]
    x_lim = MinMax(-15, 15)
    y_lim = MinMax(-15, 15)
    time_params = TimeParameters(span_sec=SIMULATION_SPAN_SEC, interval_sec=INTERVAL_SEC)
    vis = GlobalXYVisualizer(x_lim, y_lim, time_params, show_zoom=False)
    sim = EKFSLAMSimulation(
        true_landmarks,
        sigma_r=SIGMA_R,
        sigma_phi=SIGMA_PHI,
        sigma_v=SIGMA_V,
        sigma_omega=SIGMA_OMEGA,
        gate_threshold=GATE_THRESHOLD,
        max_range=MAX_RANGE,
        radius=radius_lm,
        speed=0.5,
    )
    vis.add_object(sim)
    if not show_plot:
        vis.not_show_plot()
    vis.draw()


if __name__ == "__main__":
    main()
