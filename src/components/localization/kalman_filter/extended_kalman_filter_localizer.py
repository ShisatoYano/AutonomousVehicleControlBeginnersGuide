"""
extended_kalman_filter_localizer.py

Author: Shisato Yano
"""

import sys
import numpy as np
from pathlib import Path
from math import cos, sin, sqrt, atan2, pi

sys.path.append(str(Path(__file__).absolute().parent) + "/../../state")
sys.path.append(str(Path(__file__).absolute().parent) + "/../../array")
sys.path.append(str(Path(__file__).absolute().parent) + "/../../sensors/gnss")
from state import State
from xy_array import XYArray


class ExtendedKalmanFilterLocalizer:
    """
    Self localization by Extended Kalman Filter class

    Notation used in the comments below
        x = [x, y, yaw, speed]^T   state vector (4x1)
        u = [accel, yaw rate]^T    input vector (2x1)
        z = [x, y]^T               observation (GNSS position, 2x1)
        P                          covariance matrix of the state (4x4)
        Q                          covariance matrix of the input noise (2x2)
        R                          covariance matrix of the observation noise (2x2)

    Each cycle has two steps.
        Predict: propagate the last estimate with the motion model
            x_pred = f(x_last, u)
            P_pred = F P_last F^T + G Q G^T
        Update: correct the prediction with the GNSS observation
            S = H P_pred H^T + R
            K = P_pred H^T S^-1
            x_est = x_pred + K (z - h(x_pred))
            P_est = (I - K H) P_pred

    The motion model f is State.motion_model(). The Jacobians F, G and H
    are described at _jacobian_F(), _jacobian_G() and JACOB_H.

    References
        S. Thrun, W. Burgard, D. Fox, "Probabilistic Robotics",
        MIT Press, 2005. Chapter 3 (Gaussian Filters) explains the
        Kalman filter and the Extended Kalman Filter.
        R. E. Kalman, "A New Approach to Linear Filtering and Prediction
        Problems", Journal of Basic Engineering, 82(1), 1960.
        PythonRobotics (Atsushi Sakai), Extended Kalman Filter Localization:
        https://github.com/AtsushiSakai/PythonRobotics
    """

    def __init__(self, accel_noise=0.2, yaw_rate_noise=10.0,
                 obsrv_x_noise=1.0, obsrv_y_noise=1.0, color='r'):
        """
        Constructor
        accel_noise: Standard deviation of acceleration input noise[m/s^2]
        yaw_rate_noise: Standard deviation of yaw rate input noise[deg/s]
        obsrv_x_noise: Standard deviation of x observation noise[m]
        obsrv_y_noise: Standard deviation of y observation noise[m]
        color: Color of drawing error covariance ellipse
        """
        
        self.state = np.zeros((4, 1))
        self.cov_mat = np.eye(4)
        # Q = diag(sigma_accel^2, sigma_yaw_rate^2), yaw rate in [rad/s]
        self.INPUT_NOISE_VAR_MAT = np.diag([accel_noise, np.deg2rad(yaw_rate_noise)]) ** 2
        # R = diag(sigma_x^2, sigma_y^2)
        self.OBSRV_NOISE_VAR_MAT = np.diag([obsrv_x_noise, obsrv_y_noise]) ** 2
        # Jacobian H of the observation model h(x) = [x, y]^T.
        # GNSS observes only x and y, so h is linear and H = dh/dx is constant:
        #   H = [[1, 0, 0, 0],
        #        [0, 1, 0, 0]]
        self.JACOB_H = np.array([[1, 0, 0, 0],
                                 [0, 1, 0, 0]])
        self.DRAW_COLOR = color
    
    def update(self, state, accel_mps2, yaw_rate_rps, time_s, gnss):
        """
        Function to update data
        state: Last estimated state data
        accel_mps2: Acceleration input from controller
        yaw_rate_rps: Yaw rate input from controller
        time_s: Simulation interval time[sec]
        gnss: Observation [x, y]^T from GNSS(2x1 array)[m]
        Return: Estimated state data
        """
        
        last_state = np.array([[state.get_x_m()],
                               [state.get_y_m()],
                               [state.get_yaw_rad()],
                               [state.get_speed_mps()]])
        
        # input with noise
        # u = u_org + noise
        # Note: the noise is made by multiplying the variance matrix Q (not
        # the standard deviation) by a standard normal random vector, so the
        # sampled noise is sigma^2 * N(0, 1). This is kept as it is, so the
        # behavior of the simulation does not change.
        input_org = np.array([[accel_mps2],
                              [yaw_rate_rps]])
        input_noise = self.INPUT_NOISE_VAR_MAT @ np.random.randn(2, 1)
        next_input = input_org + input_noise
        
        # predict state
        # x_pred = f(x_last, u) = A(yaw_last) x_last + B(yaw_last) u
        #   x_pred   = x + speed*cos(yaw)*t + accel*cos(yaw)*t^2/2
        #   y_pred   = y + speed*sin(yaw)*t + accel*sin(yaw)*t^2/2
        #   yaw_pred = yaw + yaw_rate*t
        #   spd_pred = speed + accel*t
        pred_state = State.motion_model(last_state, next_input, time_s)
        # F = df/dx, G = df/du
        # Note: the code evaluates them at the predicted state (pred_state),
        # not at the last state as in the textbook.
        jF = self._jacobian_F(pred_state, next_input, time_s)
        jG = self._jacobian_G(pred_state, time_s)
        Q = self.INPUT_NOISE_VAR_MAT
        last_cov = self.cov_mat
        # P_pred = F P_last F^T + G Q G^T
        # The first term propagates the last uncertainty through the motion
        # model, the second term adds the uncertainty of the input noise.
        pred_cov = jF @ last_cov @jF.T + jG @ Q @ jG.T

        # predict observation
        # z_pred = h(x_pred) = H x_pred
        # S = H P_pred H^T + R  (covariance of the innovation)
        pred_obsrv = self._observation_model(pred_state)
        jH = self.JACOB_H
        R = self.OBSRV_NOISE_VAR_MAT
        pred_obsrv_cov = jH @ pred_cov @ jH.T + R
        
        # kalman gain
        # K = P_pred H^T S^-1
        # Large R (noisy GNSS) makes K small, so the prediction is trusted.
        # Small R makes K large, so the observation is trusted.
        k = pred_cov @ jH.T @ np.linalg.inv(pred_obsrv_cov)

        # update
        # innovation: y = z - z_pred (GNSS observation minus predicted one)
        # x_est = x_pred + K y
        # P_est = P_pred - K S K^T
        #   This equals (I - K H) P_pred, because K S K^T = K H P_pred.
        inov = gnss - pred_obsrv
        est_state = pred_state + k @ inov
        est_cov = pred_cov - k @ pred_obsrv_cov @ k.T

        self.state = est_state
        self.cov_mat = est_cov

        return est_state
    
    def draw(self, axes, elems, pose):
        """
        Function to draw error covariance ellipse
        axes: Axes objects of figure
        elems: List of plot object
        pose: Vehicle's pose[x, y, yaw]
        """
        
        eig_val, eig_vec = np.linalg.eig(self.cov_mat)
        if eig_val[0] >= eig_val[1]: big_idx, small_idx = 0, 1
        else: big_idx, small_idx = 1, 0
        a, b = sqrt(3.0 * eig_val[big_idx]), sqrt(3.0 * eig_val[small_idx])
        angle = atan2(eig_vec[1, big_idx], eig_vec[0, big_idx])

        t = np.arange(0, 2 * pi + 0.1, 0.1)
        xs = [a * cos(it) for it in t]
        ys = [b * sin(it) for it in t]
        xys = np.array([xs, ys])
        xys_array = XYArray(xys)

        transformed_xys = xys_array.homogeneous_transformation(pose[0, 0], pose[1, 0], angle)
        elip_plot, = axes.plot(transformed_xys.get_x_data(), transformed_xys.get_y_data(), color=self.DRAW_COLOR)
        elems.append(elip_plot)        

    def _jacobian_F(self, state, input, time_s):
        """
        Private function to calculate jacobian F
        F = df/dx: partial derivatives of the motion model f(x, u) with
        respect to the state x = [x, y, yaw, speed]^T. The motion model is
        nonlinear in yaw, so it is linearized around the given state
        (first-order Taylor expansion). With t = time_s:
          x_next   = x + speed*cos(yaw)*t + accel*cos(yaw)*t^2/2
          y_next   = y + speed*sin(yaw)*t + accel*sin(yaw)*t^2/2
          yaw_next = yaw + yaw_rate*t
          spd_next = speed + accel*t
        Differentiating each row gives, for example,
          d x_next / d yaw   = -speed*sin(yaw)*t - accel*sin(yaw)*t^2/2
          d x_next / d speed = cos(yaw)*t
        state: Predicted state
        input: Input vector[accel, yaw rate] from controller
        time_s: Simulation interval time[sec]
        Return: Jacobian F
        """
        
        yaw = state[2, 0]
        spd = state[3, 0]
        acl = input[0, 0]
        t = time_s

        sin_yaw = sin(yaw)
        cos_yaw = cos(yaw)

        jF = np.array([[1, 0, -spd*sin_yaw*t-acl*sin_yaw*t**2/2, cos_yaw*t],
                       [0, 1, spd*cos_yaw*t+acl*cos_yaw*t**2/2, sin_yaw*t],
                       [0, 0, 1, 0],
                       [0, 0, 0, 1]])
        
        return jF
    
    def _jacobian_G(self, state, time_s):
        """
        Private function to calculate jacobian G
        G = df/du: partial derivatives of the motion model f(x, u) with
        respect to the input u = [accel, yaw rate]^T. It maps the input noise
        into the state space. Because f is linear in u, G is the same as
        matrix B in State.motion_model(). For example,
          d x_next / d accel      = cos(yaw)*t^2/2
          d yaw_next / d yaw_rate = t
          d spd_next / d accel    = t
        state: Predicted state
        time_s: Simulation interval time[sec]
        Return: Jacobian G
        """
        
        yaw = state[2, 0]
        t = time_s

        jG = np.array([[cos(yaw) * t**2 / 2, 0],
                       [sin(yaw) * t**2 / 2, 0],
                       [0, t],
                       [t, 0]])
        
        return jG
    
    def _observation_model(self, state):
        """
        Private function of observation model
        h(x) = H x = [x, y]^T: GNSS observes only the position, so yaw and
        speed are dropped by H.
        state: Predicted state
        Return Predicted observation data
        """
        
        H = np.array([[1, 0, 0, 0],
                      [0, 1, 0, 0]])
        
        x = np.array([[state[0, 0]],
                      [state[1, 0]],
                      [state[2, 0]],
                      [state[3, 0]]])
        
        return H @ x
