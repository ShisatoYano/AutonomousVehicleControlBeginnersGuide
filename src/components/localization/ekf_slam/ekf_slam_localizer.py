"""
ekf_slam_localizer.py

EKF-SLAM localizer component following the ExtendedKalmanFilterLocalizer pattern.
Jointly estimates robot pose [x, y, yaw] and landmark positions [lx, ly, ...]
using range-bearing observations.
"""

import numpy as np
from math import sqrt, pi

from motion_model import predict_robot_state, jacobian_F, jacobian_G
from observation_model import observe_landmark, build_H_matrix
from data_association import nearest_neighbor_association, POTENTIAL_DUPLICATE
from landmark_manager import augment_state, initialize_landmark_from_observation


class EKFSLAMLocalizer:
    """
    EKF-SLAM localizer: simultaneous localization and mapping via Extended Kalman Filter.
    Interface mirrors ExtendedKalmanFilterLocalizer where possible.
    """

    def __init__(self, init_x=0.0, init_y=0.0, init_yaw=0.0,
                 sigma_r=0.2, sigma_phi=0.1, sigma_v=0.15, sigma_omega=0.08,
                 gate_threshold=2.0, max_range=12.0,
                 duplicate_position_threshold=4.5, color='r'):
        self.R_obs = np.diag([sigma_r ** 2, sigma_phi ** 2])
        self.Q_input = np.diag([sigma_v ** 2, sigma_omega ** 2])
        self.sigma_r = sigma_r
        self.sigma_phi = sigma_phi
        self.gate_threshold = gate_threshold
        self.duplicate_position_threshold = duplicate_position_threshold
        self.max_range = max_range
        self.DRAW_COLOR = color

        self.mu = np.array([[init_x], [init_y], [init_yaw]])
        self.Sigma = np.eye(3) * 0.1

    def predict(self, control, dt):
        """
        EKF prediction step.
        control: (2,1) [v, omega]
        dt: time step [s]
        """
        n = self.mu.shape[0]
        robot_state = self.mu[0:3]
        robot_pred = predict_robot_state(robot_state, control, dt)
        F = jacobian_F(robot_state, control, dt)
        G = jacobian_G(robot_state, control, dt)
        Sigma_rr = self.Sigma[0:3, 0:3]
        Sigma_rr_new = F @ Sigma_rr @ F.T + G @ self.Q_input @ G.T
        self.mu[0:3] = robot_pred
        if n == 3:
            self.Sigma[0:3, 0:3] = Sigma_rr_new
        else:
            Sigma_rl = self.Sigma[0:3, 3:].copy()
            self.Sigma[0:3, 0:3] = Sigma_rr_new
            self.Sigma[0:3, 3:] = F @ Sigma_rl
            self.Sigma[3:, 0:3] = Sigma_rl.T @ F.T

    def update(self, observations):
        """
        EKF update step with data association and landmark augmentation.
        observations: list of (2,1) [range, bearing] measurements
        """
        if not observations:
            return

        n = self.mu.shape[0]
        num_landmarks = (n - 3) // 2
        pred_obs_list = []
        S_list = []
        for j in range(num_landmarks):
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
                list(range(num_landmarks)), self.gate_threshold
            )

        for (obs_idx, land_id) in matches:
            n = self.mu.shape[0]
            z = observations[obs_idx]
            if land_id >= 0:
                j = land_id
                lx = self.mu[3 + 2 * j, 0]
                ly = self.mu[3 + 2 * j + 1, 0]
                z_pred = observe_landmark(self.mu[0:3], lx, ly)
                if z_pred[0, 0] < 0.5:
                    continue
                H = build_H_matrix(self.mu[0:3], lx, ly, j, n)
                S = H @ self.Sigma @ H.T + self.R_obs
                try:
                    # K = Sigma @ H.T @ inv(S); inv() is clear for beginners; np.linalg.solve is preferred in production
                    K = self.Sigma @ H.T @ np.linalg.inv(S)
                except np.linalg.LinAlgError:
                    continue
                innov = z - z_pred
                innov[1, 0] = (innov[1, 0] + pi) % (2 * pi) - pi
                self.mu = self.mu + K @ innov
                # Standard EKF covariance update (Joseph form can be used for better numerical stability)
                IKH = np.eye(n) - K @ H
                self.Sigma = IKH @ self.Sigma
            elif land_id == POTENTIAL_DUPLICATE:
                continue
            else:
                # Cross-time duplicate check: skip if would-be position is near an existing landmark
                lm_new, _, _ = initialize_landmark_from_observation(
                    self.mu[0:3], z, None, None
                )
                lx_new, ly_new = float(lm_new[0, 0]), float(lm_new[1, 0])
                is_duplicate = False
                for j in range((self.mu.shape[0] - 3) // 2):
                    lx_j = self.mu[3 + 2 * j, 0]
                    ly_j = self.mu[3 + 2 * j + 1, 0]
                    d = sqrt((lx_new - lx_j) ** 2 + (ly_new - ly_j) ** 2)
                    if d < self.duplicate_position_threshold:
                        is_duplicate = True
                        break
                if is_duplicate:
                    continue
                self.mu, self.Sigma = augment_state(
                    self.mu, self.Sigma, z, self.mu[0:3], self.R_obs
                )

    def get_robot_state(self):
        """Returns (3,1) array [x, y, yaw]."""
        return self.mu[0:3].copy()

    def get_estimated_landmarks(self):
        """Returns list of (lx, ly) tuples for all mapped landmarks."""
        n = self.mu.shape[0]
        if n <= 3:
            return []
        return [
            (self.mu[3 + 2 * j, 0], self.mu[3 + 2 * j + 1, 0])
            for j in range((n - 3) // 2)
        ]

    def draw(self, axes, elems, pose):
        """
        Draw uncertainty circle and estimated landmarks.
        axes: Axes object
        elems: list of plot elements
        pose: (3,1) [x, y, yaw] for circle center
        """
        est_lm = self.get_estimated_landmarks()
        if est_lm:
            lx_est = [p[0] for p in est_lm]
            ly_est = [p[1] for p in est_lm]
            p, = axes.plot(lx_est, ly_est, "rx", markersize=6, label="Est. landmarks")
            elems.append(p)

        # Simple uncertainty circle (radius ~ 3-sigma from trace of pose covariance)
        Sigma_xy = self.Sigma[0:2, 0:2]
        try:
            trace_xy = np.trace(Sigma_xy) / 2.0
            r = 3.0 * sqrt(max(0.0, trace_xy))
            t = np.arange(0, 2 * pi + 0.1, 0.1)
            cx = pose[0, 0] + r * np.cos(t)
            cy = pose[1, 0] + r * np.sin(t)
            p, = axes.plot(cx, cy, color=self.DRAW_COLOR, linewidth=0.8)
            elems.append(p)
        except (np.linalg.LinAlgError, ValueError):
            pass
