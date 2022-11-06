"""
Accurate linear motion model program

x_{t+1} = x_t + v_t * cos(yaw_t + omega_t*dt/2) * dt
y_{t+1} = y_t + v_t * sin(yaw_t + omega_t*dt/2) * dt
yaw_{t+1} = yaw_t + omega_t * dt

Author: Shisato Yano
"""

import sys
import os
import numpy as np
import matplotlib.pyplot as plt
from math import sin, cos, asin

# path setting to import external module
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../vehicle")
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../../../Basis/Sources/transformation")
from four_wheels_vehicle import FourWheelsVehicle
from transformation import convert_speed_kmh_2_ms

# parameters constant
INTERVAL_SEC = 0.1
INTERVAL_MSEC = INTERVAL_SEC * 1000
TIME_LIMIT_SEC = 30
SPEED_KMH = 20
YAW_RATE_DS = 15
PLOT_WIDTH_M = 5

# flag to switch showing plot
# when this module is called for unit testing,
# this flag is set as false not to show plot
show_plot = True


class AccurateLinearMotionModel:
    """
    Class to predict 3 dimensional state (position x, y and yaw angle)
    by Accurate Linear Motion Model
    Yaw angle at center of interval time is used to calculate state
    Tips to decrease offset error by modeling
    """

    def __init__(self, front_length_m=2.0, rear_length_m=0.0, 
                 interval_sec=0.1, speed_noise_std_mps=0.0,
                 yaw_rate_noise_std_rps=0.0):
        """
        Constructor
        front_length_m: length from vehicle origin to front edge[m]
        rear_length_m: length from vehicle origin to rear edge[m]
        interval_sec: interval time between previous and current step[sec]
        speed_noise_std_mps: standard deviation included in speed input[m/sec]
        yaw_rate_noise_std_rps: standard deviation included in yaw rate input[rad/sec]
        """

        # setting parameters
        self.WHEEL_BASE_M = front_length_m + rear_length_m
        self.DT = interval_sec
        self.SPEED_NOISE_STD = speed_noise_std_mps
        self.YAW_RATE_NOISE_STD = yaw_rate_noise_std_rps

        self.matrix_A_in_state_equation()
        self.input_covariance_matrix()

    def matrix_A_in_state_equation(self):
        """
        Matrix A in state equation
        x_{t+1} = A * x_t + G * u_t
        """

        self.A = np.array([[1.0, 0.0, 0.0],
                           [0.0, 1.0, 0.0],
                           [0.0, 0.0, 1.0]])
    
    def matrix_B_in_state_equation(self, yaw_rad, yaw_rate_rps):
        """
        Matrix B in state equation
        x_{t+1} = A * x_t + B * u_t
        """

        return np.array([[cos(yaw_rad + yaw_rate_rps*self.DT/2) * self.DT, 0.0],
                         [sin(yaw_rad + yaw_rate_rps*self.DT/2) * self.DT, 0.0],
                         [0.0, self.DT]])
    
    def input_covariance_matrix(self):
        """
        System noise covariance matrix (2 x 2)
        [[speed input std^2 0.0],
         [0.0, yaw rate input std^2]]
        """

        self.Q = np.zeros((2, 2))
        self.Q[0, 0] = self.SPEED_NOISE_STD ** 2
        self.Q[1, 1] = self.YAW_RATE_NOISE_STD ** 2
    
    def jacobian_F(self, speed_mps, yaw_rad, yaw_rate_rps):
        """
        Jacobian matrix of Motion model
        about state [x, y, yaw]

        dx/dx = 1, dx/dy = 0, dx/dyaw = -v*sin(yaw + yaw_rate*dt/2)*dt
        dy/dx = 0, dy/dy = 1, dy/dyaw = v*cos(yaw + yaw_rate*dt/2)*dt
        dyaw/dx = 0, dyaw/dy = 0, dyaw/dyaw = 1 
        """

        return np.array([[1.0, 0.0, -speed_mps*sin(yaw_rad + yaw_rate_rps*self.DT/2)*self.DT],
                         [0.0, 1.0, speed_mps*cos(yaw_rad + yaw_rate_rps*self.DT/2)*self.DT],
                         [0.0, 0.0, 1.0]])
    
    def jacobian_G(self, speed_mps, yaw_rad, yaw_rate_rps):
        """
        Jacobian matrix of Motion model
        about input [speed, yaw rate]
        Speed: v
        Yaw rate: ω

        dx/dv = cos(yaw + yaw_rate*dt/2)*dt, dx/dω = dt/2 * v * -sin(yaw + yaw_rate*dt/2)*dt
        dy/dv = sin(yaw + yaw_rate*dt/2)*dt, dy/dω = dt/2 * v * cos(yaw + yaw_rate*dt/2)*dt
        dyaw/dv = 0, dyaw/dω = dt
        """

        return np.array([[cos(yaw_rad+yaw_rate_rps*self.DT/2)*self.DT, -self.DT*speed_mps*sin(yaw_rad + yaw_rate_rps*self.DT/2)*self.DT/2],
                         [sin(yaw_rad+yaw_rate_rps*self.DT/2)*self.DT, self.DT*speed_mps*cos(yaw_rad + yaw_rate_rps*self.DT/2)*self.DT/2],
                         [0.0, self.DT]])

    def calculate_state(self, x, u):
        """
        Function of state equation
        This is used to predict vehicle's position and velocity at next time
        by using acceleration input
        x_{t+1} = F * x_t + G * u_t

        x: state vector [x[m], y[m], yaw[rad]]' at previous step
        u: input vector [speed[m/s], yaw rate[rad/s]]' at current step
        """

        B = self.matrix_B_in_state_equation(x[2, 0], u[1, 0])
        x = self.A @ x + B @ u

        # calculate steering angle from speed and yaw rate
        # zero division error will happen when speed is zero
        steer_rad = asin(self.WHEEL_BASE_M * u[1, 0] / u[0, 0])

        return x, steer_rad
    
    def calculate_covariance(self, x, p):
        """
        Function to predict covariance of position and velocity
        p_{t+1} = F * p^_t * F^T + G * Q * G^T

        x: state vector [x[m], y[m], yaw[rad]]' at previous step
        p: estimated covariance at previous time, p^_t
        """

        F = self.jacobian_F(u[0, 0], x[2, 0], u[1, 0])
        G = self.jacobian_G(u[0, 0], x[2, 0], u[1, 0])
        
        return F @ p @ F.T + G @ self.Q @ G.T


def calculate_input(u, t):
    """
    Function to calculate input [speed[m/sex], yaw rate[rad/sec]]

    u: input vector [speed[m/sex], yaw rate[rad/sec]]
    t: elapsed time[sec]
    """

    if t <= TIME_LIMIT_SEC/2:
        u[1, 0] = np.deg2rad(YAW_RATE_DS)
    else:
        u[1, 0] = -np.deg2rad(YAW_RATE_DS)
    u[0, 0] = convert_speed_kmh_2_ms(SPEED_KMH)

    return u


def main():
    """
    Main function
    """

    print(__file__ + " + start!!")

    # clear all figures
    plt.clf()

    # initialize figure
    ax = plt.subplot(1, 1, 1)
    ax.set_xlabel("X[m]")
    ax.set_ylabel("Y[m]")
    ax.set_aspect("equal")
    ax.grid(True)

    # generate instances
    almm = AccurateLinearMotionModel(interval_sec=INTERVAL_SEC) # motion model
    fwv = FourWheelsVehicle(ax) # vehicle drawing

    # initialize data
    elapsed_time_sec = 0.0
    state_vector = np.zeros((3, 1))
    input_vector = np.zeros((2, 1))

    # simulation loop
    while TIME_LIMIT_SEC >= elapsed_time_sec:
        # calculate input, speed and yaw rate
        input_vector = calculate_input(u=input_vector, t=elapsed_time_sec)

        # calculate state by motion model
        state_vector, steer_rad = almm.calculate_state(x=state_vector, u=input_vector)

        # draw vehicle
        fwv.draw(state_vector[0, 0], state_vector[1, 0], np.rad2deg(state_vector[2, 0]), np.rad2deg(steer_rad))

        # update plot range
        center_x_m = state_vector[0, 0]
        center_y_m = state_vector[1, 0]
        ax.set_xlim([center_x_m - PLOT_WIDTH_M, center_x_m + PLOT_WIDTH_M])
        ax.set_ylim([center_y_m - PLOT_WIDTH_M, center_y_m + PLOT_WIDTH_M])

        # increment time
        elapsed_time_sec += INTERVAL_SEC

        # pause temporarily to show animation
        # only when flag to show plot is true
        if show_plot: plt.pause(INTERVAL_SEC)

    return True


# execute main process
if __name__ == "__main__":
    main()
