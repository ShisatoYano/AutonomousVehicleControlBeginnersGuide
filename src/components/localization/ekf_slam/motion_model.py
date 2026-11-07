"""
motion_model.py

Jacobians for EKF-SLAM prediction step. Mean propagation uses State.motion_model
(see state.py): robot state [x, y, yaw, speed], control [accel, yaw_rate].
"""

from math import cos, sin

import numpy as np


def jacobian_F(pred_state, control, time_s):
    """
    Jacobian of State.motion_model w.r.t. state [x, y, yaw, speed].
    pred_state: (4, 1) predicted state after State.motion_model
    control: (2, 1) [accel_mps2, yaw_rate_rps]
    time_s: time step [s]
    Returns: (4, 4) Jacobian F (matches ExtendedKalmanFilterLocalizer._jacobian_F)
    """
    yaw = pred_state[2, 0]
    spd = pred_state[3, 0]
    acl = control[0, 0]
    t = time_s

    sin_yaw = sin(yaw)
    cos_yaw = cos(yaw)

    jF = np.array(
        [
            [1, 0, -spd * sin_yaw * t - acl * sin_yaw * t**2 / 2, cos_yaw * t],
            [0, 1, spd * cos_yaw * t + acl * cos_yaw * t**2 / 2, sin_yaw * t],
            [0, 0, 1, 0],
            [0, 0, 0, 1],
        ]
    )

    return jF


def jacobian_G(pred_state, time_s):
    """
    Jacobian of State.motion_model w.r.t. control [accel, yaw_rate].
    pred_state: (4, 1) predicted state
    time_s: time step [s]
    Returns: (4, 2) Jacobian G (matches ExtendedKalmanFilterLocalizer._jacobian_G)
    """
    yaw = pred_state[2, 0]
    t = time_s

    jG = np.array(
        [
            [cos(yaw) * t**2 / 2, 0],
            [sin(yaw) * t**2 / 2, 0],
            [0, t],
            [t, 0],
        ]
    )

    return jG
