"""
Linear Kalman Filter sample program to estimate 1 dimenonal position.

A vehicle drives at a velocity, v.
A LiDAR observes a distance to the vehicle, z.
The velocity and the observation include Gaussian noise.
This system can predict the position with the velocity and
observe it with the LiDAR. And then, the position can be estimated
by Kalman Filter.

Author: Shisato Yano
"""

import numpy as np
import matplotlib.pyplot as plt

# parameters
INTERVAL_SEC = 0.01
TIME_LIMIT_SEC = 10.0
INPUT_VELOCITY_MS = 10.0
INPUT_NOISE_VARIANCE = 10.0
OBSERVATION_NOISE_VARIANCE = 5.0


class LinearKalmanFilter1D:
    """
    Class to estimate 1 dimensional position by Linear Kalman Filter.
    """

    def __init__(self, interval_sec, input_velocity_ms, input_noise_variance, observation_noise_variance):
        """
        Constructor
        interval_sec: interval time[s] to update this simulation
        input_velocity_ms: input velocity[m/s] into state equation
        input_noise_variance: variance of noise included in input velocity
        observation_noise_variance: variance of noise inculded in observation
        """

        # set inputs as parameters
        self.DT = interval_sec
        self.V = input_velocity_ms
        self.Q = input_noise_variance
        self.R = observation_noise_variance

        # define parameters for state equation
        self.F = 1.0
        self.G = self.DT

        # define parameter for observation equation
        self.H = 1.0
    
    def state_equation(self, x_prev, u):
        """
        Function of state equation
        This is used to predict vehicle's position at next time
        by using velocity input
        x_{t+1} = F * x_t + G * u_t

        x_prev: position at previous time, x_t
        u: velocity input, u_t
        """

        x_next = self.F * x_prev + self.G * u
        return x_next
    
    def predict_position(self, x_prev, u):
        """
        Function to predict position at next time
        Predict by inputing velocity into linear motion model
        Velocity input is including gaussian noise

        x_prev: position at previous time, x_t
        u: velocity input including gaussian noise, u_t
        """

        x_pred = self.state_equation(x_prev, u)
        return x_pred

    def observation(self, x_prev):
        """
        Function to generate observation data
        Simulate position data observed by LiDAR
        This data is including gaussian noise

        x_prev: position at previous time
        """

        z = self.state_equation(x_prev, self.V) + np.random.randn() * self.R
        return z
    
    def observation_equation(self, x_pred):
        """
        Function of observation equation
        This is used to predict observation data at next time
        z_{t+1} = H * x_{t+1}

        x_pred: predicted position by state equation, x_{t+1}
        """
        
        z_pred = self.H * x_pred
        return z_pred
    
    def predict_variance(self, p_prev):
        """
        Function to predict variance
        p_{t+1} = F^2 * p^_t + G^2 * Q

        p_prev: estimated variance at previous time, p^_t
        """
        
        p_pred = self.F**2 * p_prev + self.G**2 * self.Q
        return p_pred
    
    def calculate_innovation(self, z, x_pred):
        """
        Function to calculate observation prediction error variance
        dz = z_{t+1} - H * x_{t+1}

        x_pred: predicted position with previous data, x_{t+1}
        """

        delta_z = z - self.observation_equation(x_pred)
        return delta_z
    
    def calculate_obsrv_pred_err_var(self, p_pred):
        """
        Function to calculate observation prediction error variance
        s_{t+1} = H^2 * p_{t+1} + R

        p_pred: predicted variance with previous data, p_{t+1}
        """

        p_obsrv_pred_err = self.H**2 * p_pred + self.R
        return p_obsrv_pred_err
    
    def calculate_kalman_gain(self, p_pred, p_obsrv_pred_err):
        """
        Function to calculate kalman gain
        k_{t+1} = p_{t+1} * H * s_{t+1}^{-1}

        p_pred: predicted variance with previous data, p_{t+1}
        p_obsrv_pred_err: observation prediction error variance, s_{t+1}
        """

        kalman_gain = (p_pred * self.H) / p_obsrv_pred_err
        return kalman_gain
    
    def update_position(self, x_pred, delta_z, kalman_gain):
        """
        Function to update position
        x^_{t+1} = x_{t+1} + k_{t+1} * dz

        x_pred: predicted position with previous data, x^_{t+1}
        delta_z: difference between observation and predicted observation, dz
        kalman_gain: kalman gain to update position, k_{t+1}
        """

        x_updated = x_pred + kalman_gain * delta_z
        return x_updated
    
    def update_variance(self, p_pred, kalman_gain, p_obsrv_pred_err):
        """
        Function to update variane
        p^_{t+1} = p_{t+1} - k_{t+1}^2 * s_{t+1}

        p_pred: predicted variance with previous data, p^_{t+1}
        kalman_gain: kalman gain to update position, k_{t+1}
        p_obsrv_pred_err: observation prediction error variance, s_{t+1}
        """

        p_updated = p_pred - kalman_gain**2 * p_obsrv_pred_err
        return p_updated
    
    def estimate_position(self, x_prev, p_prev, u, z):
        """
        Function to estimate position and variance by linear kalman filter
        Step 1: update position and variance with previous data
        Step 2: update position and variance with current observation

        x_prev: estimated position at previous time
        p_prev: estimated variance at previous time
        u: velocity input including gaussian noise
        z: observed position at current time including gaussian noise
        """

        # update with previous data
        x_pred = self.predict_position(x_prev, u)
        p_pred = self.predict_variance(p_prev)

        # update with current observation
        delta_z = self.calculate_innovation(z, x_pred)
        p_obsrv_pred_err = self.calculate_obsrv_pred_err_var(p_pred)
        kalman_gain = self.calculate_kalman_gain(p_pred, p_obsrv_pred_err)
        x_updated = self.update_position(x_pred, delta_z, kalman_gain)
        p_updated = self.update_variance(p_pred, kalman_gain, p_obsrv_pred_err)

        return x_updated, p_updated


def main():
    """
    main function
    """

    print(__file__ + " start!!")

    # generate instance of kalman filter
    kf = LinearKalmanFilter1D(interval_sec=INTERVAL_SEC,
                              input_velocity_ms=INPUT_VELOCITY_MS,
                              input_noise_variance=INPUT_NOISE_VARIANCE,
                              observation_noise_variance=OBSERVATION_NOISE_VARIANCE)
    
    # initialize data
    elapsed_time_sec, true_pos, obsrv_pos, pred_pos = 0.0, 0.0, 0.0, 0.0
    time_list, true_pos_list, obsrv_pos_list, pred_pos_list = [], [], [], []
    est_pos, est_var = 0.0, 1.0
    est_pos_list, est_var_list = [], []

    # initialize plot
    fig = plt.figure()
    # position plot
    ax_position = fig.add_subplot(1, 2, 1)
    ax_position.set_xlabel("Time[s]")
    ax_position.set_ylabel("Position[m]")
    ax_position.grid(True)
    # variance plot
    ax_variance = fig.add_subplot(1, 2, 2)
    ax_variance.set_xlabel("Time[s]")
    ax_variance.set_ylabel("Variance")
    ax_variance.grid(True)

    # simulate
    while TIME_LIMIT_SEC >= elapsed_time_sec:
        # decide input velocity
        input_velocity = INPUT_VELOCITY_MS + np.random.randn() * INPUT_NOISE_VARIANCE
        if input_velocity < 0.0: input_velocity = 0.0

        # calculate data
        obsrv_pos = kf.observation(x_prev=true_pos)
        true_pos = kf.state_equation(x_prev=true_pos, u=INPUT_VELOCITY_MS)
        pred_pos = kf.predict_position(x_prev=pred_pos, u=input_velocity)
        est_pos, est_var = kf.estimate_position(x_prev=est_pos, p_prev=est_var, u=input_velocity, z=obsrv_pos)

        # record data
        time_list.append(elapsed_time_sec)
        obsrv_pos_list.append(obsrv_pos)
        true_pos_list.append(true_pos)
        pred_pos_list.append(pred_pos)
        est_pos_list.append(est_pos)
        est_var_list.append(est_var)

        # increment time
        elapsed_time_sec += INTERVAL_SEC
    
    # plot data
    ax_position.plot(time_list, obsrv_pos_list, color='g', lw=2, label="Observed Position")
    ax_position.plot(time_list, pred_pos_list, color='m', lw=2, label="Predicted Position")
    ax_position.plot(time_list, true_pos_list, color='b', lw=2, label="True Position")
    ax_position.plot(time_list, est_pos_list, color='r', lw=2, label="Estimated Position")
    ax_position.legend()
    ax_variance.plot(time_list, est_var_list, color='r', lw=2, label="Estimated Variance")
    ax_variance.legend()
    fig.tight_layout()
    plt.show()


# execute main process
if __name__ == "__main__":
    main()
