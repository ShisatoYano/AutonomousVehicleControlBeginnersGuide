"""
motion_model.py

Velocity motion model and Jacobians for EKF-SLAM prediction step.
Robot state: [x, y, yaw]. Control input: [v, omega].
"""

import numpy as np
from math import cos, sin


def predict_robot_state(robot_state, control, dt):
    """
    Predict next robot pose using velocity motion model.
    robot_state: (3, 1) array [x, y, yaw]
    control: (2, 1) array [v, omega]
    dt: time step [s]
    Returns: (3, 1) predicted state
    """
    x, y, yaw = robot_state[0, 0], robot_state[1, 0], robot_state[2, 0]
    v, omega = control[0, 0], control[1, 0]
    yaw_new = yaw + omega * dt
    # avoid singularities when omega is near zero
    if abs(omega) < 1e-6:
        x_new = x + v * cos(yaw) * dt
        y_new = y + v * sin(yaw) * dt
    else:
        x_new = x + (v / omega) * (sin(yaw_new) - sin(yaw))
        y_new = y + (v / omega) * (-cos(yaw_new) + cos(yaw))
    return np.array([[x_new], [y_new], [yaw_new]])


def jacobian_F(robot_state, control, dt):
    """
    Jacobian of motion model w.r.t. robot state [x, y, yaw].
    robot_state: (3, 1), control: (2, 1), dt: float
    Returns: (3, 3) Jacobian F
    """
    yaw = robot_state[2, 0]
    v, omega = control[0, 0], control[1, 0]
    if abs(omega) < 1e-6:
        dx_dyaw = -v * sin(yaw) * dt
        dy_dyaw = v * cos(yaw) * dt
    else:
        yaw_new = yaw + omega * dt
        dx_dyaw = (v / omega) * (cos(yaw_new) - cos(yaw))
        dy_dyaw = (v / omega) * (sin(yaw_new) - sin(yaw))
    F = np.array([
        [1, 0, dx_dyaw],
        [0, 1, dy_dyaw],
        [0, 0, 1]
    ])
    return F


def jacobian_G(robot_state, control, dt):
    """
    Jacobian of motion model w.r.t. control [v, omega].
    robot_state: (3, 1), control: (2, 1), dt: float
    Returns: (3, 2) Jacobian G
    """
    yaw = robot_state[2, 0]
    v, omega = control[0, 0], control[1, 0]
    if abs(omega) < 1e-6:
        dx_dv = cos(yaw) * dt
        dy_dv = sin(yaw) * dt
        dx_domega = 0.0
        dy_domega = 0.0
    else:
        yaw_new = yaw + omega * dt
        dx_dv = (sin(yaw_new) - sin(yaw)) / omega
        dy_dv = (-cos(yaw_new) + cos(yaw)) / omega
        dx_domega = (v / (omega ** 2)) * (sin(yaw) - sin(yaw_new) + omega * cos(yaw_new) * dt)
        dy_domega = (v / (omega ** 2)) * (cos(yaw_new) - cos(yaw) + omega * sin(yaw_new) * dt)
    G = np.array([
        [dx_dv, dx_domega],
        [dy_dv, dy_domega],
        [0, dt]
    ])
    return G
