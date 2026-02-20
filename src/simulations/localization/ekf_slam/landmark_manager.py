"""
landmark_manager.py

State vector augmentation and new landmark initialization for EKF-SLAM.
"""

import numpy as np
from math import cos, sin


def initialize_landmark_from_observation(robot_state, z, G_robot, G_landmark):
    """
    Compute landmark position (lx, ly) from range-bearing observation and robot pose.
    robot_state: (3, 1) [x, y, yaw]
    z: (2, 1) [range, bearing]
    G_robot: (2, 3) Jacobian of [lx, ly] w.r.t. robot state (for covariance)
    G_landmark: (2, 2) Jacobian of [lx, ly] w.r.t. [r, phi]
    Returns: (2, 1) landmark position, and G_robot, G_landmark for covariance update.
    """
    x, y, yaw = robot_state[0, 0], robot_state[1, 0], robot_state[2, 0]
    r, phi = z[0, 0], z[1, 0]
    lx = x + r * cos(yaw + phi)
    ly = y + r * sin(yaw + phi)
    # Jacobian of (lx, ly) w.r.t. (x, y, yaw): dlx/dx=1, dlx/dy=0, dlx/dyaw=-r*sin(yaw+phi)
    G_robot = np.array([
        [1, 0, -r * sin(yaw + phi)],
        [0, 1, r * cos(yaw + phi)]
    ])
    G_landmark = np.array([
        [cos(yaw + phi), -r * sin(yaw + phi)],
        [sin(yaw + phi), r * cos(yaw + phi)]
    ])
    return np.array([[lx], [ly]]), G_robot, G_landmark


def augment_state(mu, Sigma, z, robot_state, R_obs):
    """
    Augment state vector and covariance with a new landmark from observation z.
    mu: (n, 1) state vector [x, y, yaw, l1_x, l1_y, ...]
    Sigma: (n, n) covariance
    z: (2, 1) [range, bearing]
    robot_state: (3, 1) current robot pose from mu
    R_obs: (2, 2) observation noise covariance
    Returns: mu_new (n+2, 1), Sigma_new (n+2, n+2)
    """
    lm, G_robot, G_landmark = initialize_landmark_from_observation(robot_state, z, None, None)
    n = mu.shape[0]
    # Cross covariance: Sigma_xy = Sigma[0:3, :] for robot; we need Sigma_robot_rest
    Sigma_rr = Sigma[0:3, 0:3]
    Sigma_rl = G_robot @ Sigma_rr @ G_robot.T + G_landmark @ R_obs @ G_landmark.T
    Sigma_xr = Sigma[:, 0:3]  # (n, 3)
    Sigma_rx = Sigma[0:3, :]  # (3, n)
    # New landmark covariance and cross terms
    sigma_new_lm = Sigma_xr @ G_robot.T  # (n, 2)
    sigma_new_lm_T = sigma_new_lm.T       # (2, n)
    Sigma_ll = G_robot @ Sigma_rr @ G_robot.T + G_landmark @ R_obs @ G_landmark.T
    mu_new = np.vstack([mu, lm])
    Sigma_new = np.block([
        [Sigma, sigma_new_lm],
        [sigma_new_lm_T, Sigma_ll]
    ])
    return mu_new, Sigma_new
