"""
Linear Kalman Filter sample program to estimate 2 dimensional position and velocity.

A vehicle drives at a velocity, v.
The velocity, v is accelerated by acceleration, a.
A LiDAR observes a distance to the vehicle, z.
The acceleration, a and distance, z include Gussian noise.
This system can predict the position and velocity with the acceleration
and observe the distance with the LiDAR. And then, the position and velocity
can be estimated by Kalman Filter.

Author: Shisato Yano
"""

import numpy as np
import matplotlib.pyplot as plt

# parameters
INTERVAL_SEC = 0.01
TIME_LIMIT_SEC = 10.0
TIME_LIMIT_ACCEL_SEC = 2.0
INPUT_ACCEL_MS2 = 5.0
INPUT_NOISE_VARIANCE = 0.5
OBSERVATION_NOISE_VARIANCE = 5.0


class MotionModel:
    """
    Class to predict 2 dimensional state (position and velocity) by Linear Motion Model
    """

    def __init__(self, interval_sec, input_noise_variance):
        """
        Constructor
        interval_sec: interval time[s] to update this simulation
        input_noise_variance: variance of noise included in input acceleration
        """

        # set inputs as parameters
        self.DT = interval_sec
        self.Q = input_noise_variance

        self.define_matrix_in_state_equation()
    
    def define_matrix_in_state_equation(self):
        self.F = np.matrix([[1.0, self.DT],
                           [0.0, 1.0]])
        
        self.G = np.matrix([(self.DT ** 2)/2, self.DT]).T
    
    def calculate_state(self, x_prev, u):
        """
        Function of state equation
        This is used to predict vehicle's position and velocity at next time
        by using acceleration input
        x_{t+1} = F * x_t + G * u_t

        x_prev: position and velocity at previous time, x_t
        u: acceleration input, u_t
        """

        x_next = self.F * x_prev + self.G * u
        return x_next
    
    def calculate_covariance(self, p_prev):
        """
        Function to predict covariance of position and velocity
        p_{t+1} = F * p^_t * F^T + G * Q * G^T

        p_prev: estimated covariance at previous time, p^_t
        """

        p_next = self.F * p_prev * self.F.T + self.G * self.Q * self.G.T
        return p_next


class ObservationModel:
    """
    Class to simulate position info obseved by LiDAR
    and to predict observation at predicted position, x_{t+1}
    """

    def __init__(self, observation_noise_variance):
        """
        Constructor
        observation_noise_variance: variance of noise included in observation
        """

        # set input as parameter
        self.R = observation_noise_variance

        # define matrix in observation equation
        self.H = np.matrix([1.0, 0.0])
    
    def observe(self, x_true):
        """
        Function to simulate observation by including noise variance
        parameter, R
        x_true: true position
        """

        return x_true[0, 0] + np.random.randn() * self.R
    
    def calculate_observation(self, x_pred):
        """
        Function to predict observation at predicted position
        x_pred: predicted position by state equation, x_{t+1}
        """

        z_pred = self.H * x_pred
        return z_pred


class LinearKalmanFilter2D:
    """
    Class to estimate 2 dimensional state (position and velocity) by Linear Kalman Filter
    """

    def __init__(self, motion_model, observation_model):
        """
        Constructor
        motion_model: motion model to predict state
        observation_model: model to predict observation at predicted position
        """

        self.mm = motion_model
        self.om = observation_model
    
    def calculate_innovation(self, z, x_pred):
        """
        Function to calculate observation prediction error variance
        dz = z_{t+1} - H * x_{t+1}

        x_pred: predicted position with previous data, x_{t+1}
        """

        delta_z = z - self.om.calculate_observation(x_pred)
        return delta_z
    
    def calculate_obsrv_pred_err_cov(self, p_pred):
        """
        Function to calculate observation prediction error variance
        s_{t+1} = H * p_{t+1} * H^T + R

        p_pred: predicted variance with previous data, p_{t+1}
        """

        p_obsrv_pred_err = self.om.H * p_pred * self.om.H.T + self.om.R
        return p_obsrv_pred_err
    
    def calculate_kalman_gain(self, p_pred, p_obsrv_pred_err):
        """
        Function to calculate kalman gain
        k_{t+1} = p_{t+1} * H^T * s_{t+1}^{-1}

        p_pred: predicted variance with previous data, p_{t+1}
        p_obsrv_pred_err: observation prediction error variance, s_{t+1}
        """

        kalman_gain = p_pred * self.om.H.T * np.linalg.inv(p_obsrv_pred_err)
        return kalman_gain
    
    def calculate_state(self, x_pred, delta_z, kalman_gain):
        """
        Function to calculate position and velocity
        x^_{t+1} = x_{t+1} + k_{t+1} * dz

        x_pred: predicted position and velocity with previous data, x^_{t+1}
        delta_z: difference between observation and predicted observation, dz
        kalman_gain: kalman gain to update position and velocity, k_{t+1}
        """

        x_updated = x_pred + kalman_gain * delta_z
        return x_updated
    
    def calculate_covariance(self, p_pred, kalman_gain, p_obsrv_pred_err):
        """
        Function to calculate covariane of position and velocity
        p^_{t+1} = p_{t+1} - k_{t+1} * s_{t+1} * k_{t+1}^T

        p_pred: predicted covariance with previous data, p^_{t+1}
        kalman_gain: kalman gain to calculate position and velocity, k_{t+1}
        p_obsrv_pred_err: observation prediction error variance, s_{t+1}
        """

        p_updated = p_pred - kalman_gain * p_obsrv_pred_err * kalman_gain.T
        return p_updated

    def estimate_state(self, x_prev, p_prev, u, z):
        """
        Function to estimate state and covariance by linear kalman filter
        Step 1: update state and covariance with previous data
        Step 2: update state and covariance with current observation

        x_prev: estimated state at previous time
        p_prev: estimated covariance at previous time
        u: acceleration input including gaussian noise
        z: observed position at current time including gaussian noise
        """

        # update with previous data
        x_pred = self.mm.calculate_state(x_prev, u)
        p_pred = self.mm.calculate_covariance(p_prev)

        # update with current observation
        delta_z = self.calculate_innovation(z, x_pred)
        p_obsrv_pred_err = self.calculate_obsrv_pred_err_cov(p_pred)
        kalman_gain = self.calculate_kalman_gain(p_pred, p_obsrv_pred_err)
        x_updated = self.calculate_state(x_pred, delta_z, kalman_gain)
        p_updated = self.calculate_covariance(p_pred, kalman_gain, p_obsrv_pred_err)

        return x_updated, p_updated


def decide_input_accel(elapsed_time_sec):
    """
    Function to decide acceleration input
    Increasing velocity until time limit passed
    elapsed_time_sec: current elapsed time[sec]
    """

    if elapsed_time_sec <= TIME_LIMIT_ACCEL_SEC: input_accel_ms2 = INPUT_ACCEL_MS2
    else: input_accel_ms2 = 0.0

    return input_accel_ms2


def add_noise_input_accel(input_accel_ms2):
    """
    Function to generate acceleration input 
    including gaussian noise
    input_accel_ms2: true acceleration input
    """

    input_accel_noise_ms2 = input_accel_ms2 + np.random.randn() * INPUT_NOISE_VARIANCE
    if input_accel_noise_ms2 < 0.0: return 0.0
    return input_accel_noise_ms2


def main():
    """
    main function
    """

    print(__file__ + " start!!")

    # generate instances
    mm = MotionModel(interval_sec=INTERVAL_SEC, input_noise_variance=INPUT_NOISE_VARIANCE)
    om = ObservationModel(observation_noise_variance=OBSERVATION_NOISE_VARIANCE)
    kf = LinearKalmanFilter2D(motion_model=mm, observation_model=om)

    # initialize data
    input_accel_ms2 = 0.0
    elapsed_time_sec, time_list = 0.0, []
    true_state, true_pos_list, true_vel_list = np.matrix(np.zeros((2, 1))), [], []
    pred_state, pred_pos_list, pred_vel_list = np.matrix(np.zeros((2, 1))), [], []
    obsrv, obsrv_list = 0.0, []
    est_state = np.matrix(np.zeros((2, 1)))
    est_cov = np.matrix([[OBSERVATION_NOISE_VARIANCE, 0.0], 
                        [0.0, INPUT_NOISE_VARIANCE]])
    est_pos_list, est_vel_list, est_pos_cov_list, est_vel_cov_list = [], [], [], []

    # initialize plot
    fig = plt.figure()
    # position plot
    ax_position = fig.add_subplot(2, 2, 1)
    ax_position.set_xlabel("Time[s]")
    ax_position.set_ylabel("Position[m]")
    ax_position.grid(True)
    # position variance plot
    ax_pos_var = fig.add_subplot(2, 2, 2)
    ax_pos_var.set_xlabel("Time[s]")
    ax_pos_var.set_ylabel("Pos Var")
    ax_pos_var.grid(True)
    # velocity plot
    ax_veloicity = fig.add_subplot(2, 2, 3)
    ax_veloicity.set_xlabel("Time[s]")
    ax_veloicity.set_ylabel("Velocity[m/s]")
    ax_veloicity.grid(True)
    # velocity variance plot
    ax_vel_var = fig.add_subplot(2, 2, 4)
    ax_vel_var.set_xlabel("Time[s]")
    ax_vel_var.set_ylabel("Vel Var")
    ax_vel_var.grid(True)

    # simulate
    while TIME_LIMIT_SEC >= elapsed_time_sec:
        input_accel_ms2 = decide_input_accel(elapsed_time_sec=elapsed_time_sec)
        input_accel_noise_ms2 = add_noise_input_accel(input_accel_ms2)

        # calculate data
        obsrv = om.observe(true_state)
        true_state = mm.calculate_state(x_prev=true_state, u=input_accel_ms2)
        pred_state = mm.calculate_state(x_prev=pred_state, u=input_accel_noise_ms2)
        est_state, est_cov = kf.estimate_state(x_prev=est_state, p_prev=est_cov, 
                                               u=input_accel_noise_ms2, z=obsrv)

        # record data
        time_list.append(elapsed_time_sec)
        obsrv_list.append(obsrv)
        true_pos_list.append(true_state[0, 0]), true_vel_list.append(true_state[1, 0])
        pred_pos_list.append(pred_state[0, 0]), pred_vel_list.append(pred_state[1, 0])
        est_pos_list.append(est_state[0, 0]), est_vel_list.append(est_state[1, 0])
        est_pos_cov_list.append(est_cov[0, 0]), est_vel_cov_list.append(est_cov[1, 1])

        # increment time
        elapsed_time_sec += INTERVAL_SEC
    
    # plot data
    ax_position.plot(time_list, obsrv_list, color='g', lw=2, label="Observed Position")
    ax_position.plot(time_list, pred_pos_list, color='m', lw=2, label="Pred Position")
    ax_position.plot(time_list, true_pos_list, color='b', lw=2, label="True Position")
    ax_position.plot(time_list, est_pos_list, color='r', lw=2, label="Estimated Position")
    ax_position.legend()
    ax_veloicity.plot(time_list, pred_vel_list, color='m', lw=2, label="Pred Velocity")
    ax_veloicity.plot(time_list, true_vel_list, color='b', lw=2, label="True Velocity")
    ax_veloicity.plot(time_list, est_vel_list, color='r', lw=2, label="Estimated Velocity")
    ax_veloicity.legend()
    ax_pos_var.plot(time_list, est_pos_cov_list, color='r', lw=2, label="Est Pos Variance")
    ax_pos_var.legend()
    ax_vel_var.plot(time_list, est_vel_cov_list, color='r', lw=2, label="Est Vel Variance")
    ax_vel_var.legend()
    fig.tight_layout()
    plt.show()


# execute main process
if __name__ == "__main__":
    main()
