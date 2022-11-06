"""
Linear motion model comparison program

Author: Shisato Yano
"""

import sys
import os
import matplotlib.pyplot as plt
import numpy as np

# path setting to import external module
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../vehicle")
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../../../Basis/Sources/transformation")
from four_wheels_vehicle import FourWheelsVehicle
from transformation import convert_speed_kmh_2_ms
from linear_motion_model import LinearMotionModel
from accurate_linear_motion_model import AccurateLinearMotionModel

# parameters constant
INTERVAL_SEC = 0.1
INTERVAL_MSEC = INTERVAL_SEC * 1000
TIME_LIMIT_SEC = 30
SPEED_KMH = 20
YAW_RATE_DS = 15

# flag to switch showing plot
# when this module is called for unit testing,
# this flag is set as false not to show plot
show_plot = True


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
    ax.set_title("Blue:Normal Red:Accurate")
    ax.grid(True)

    # generate instance
    # motion model
    lmm = LinearMotionModel(interval_sec=INTERVAL_SEC) # normal
    almm = AccurateLinearMotionModel(interval_sec=INTERVAL_SEC) # accurate
    # vehicle drawing
    lmm_fwv = FourWheelsVehicle(ax, color='b') # normal model is colored blue
    almm_fwv = FourWheelsVehicle(ax, color='r') # accurate model is colored red
    
    # initialize data
    elapsed_time_sec = 0.0
    state_normal, state_accurate = np.zeros((3, 1)), np.zeros((3, 1))
    input_vector = np.zeros((2, 1))
    x_lmm_all, y_lmm_all = [], [] # position data list for normal
    x_almm_all, y_almm_all = [], [] # position data list for accurate

    # simulation loop
    while TIME_LIMIT_SEC >= elapsed_time_sec:
        # calculate input, speed and yaw rate
        input_vector = calculate_input(u=input_vector, t=elapsed_time_sec)

        # calculate state by normal motion model
        state_normal, steer_lmm_rad = lmm.calculate_state(x=state_normal, u=input_vector)
        x_lmm_all.append(state_normal[0, 0]), y_lmm_all.append(state_normal[1, 0]) # record data
        
        # calculate state by accurate motion model
        state_accurate, steer_almm_rad = almm.calculate_state(x=state_accurate, u=input_vector)
        x_almm_all.append(state_accurate[0, 0]), y_almm_all.append(state_accurate[1, 0])

        # draw vehicle by normal model
        lmm_fwv.draw(state_normal[0, 0], state_normal[1, 0], np.rad2deg(state_normal[2, 0]), np.rad2deg(steer_lmm_rad))

        # draw vehicle by accurate model
        almm_fwv.draw(state_accurate[0, 0], state_accurate[1, 0], np.rad2deg(state_accurate[2, 0]), np.rad2deg(steer_almm_rad))

        # update plot range
        ax.set_xlim([(state_normal[0, 0] + state_accurate[0, 0])/2 - 5, (state_normal[0, 0] + state_accurate[0, 0])/2 + 5])
        ax.set_ylim([(state_normal[1, 0] + state_accurate[1, 0])/2 - 5, (state_normal[1, 0] + state_accurate[1, 0])/2 + 5])

        # increment time
        elapsed_time_sec += INTERVAL_SEC

        # pause temporarily to show animation
        # only when flag to show plot is true
        if show_plot: plt.pause(INTERVAL_SEC)
    
    # show calculated trejectory by both models and compare
    plt.plot(x_lmm_all, y_lmm_all, ".b")
    plt.plot(x_almm_all, y_almm_all, ".r")
    plt.xlabel("X[m]")
    plt.ylabel("Y[m]")
    plt.axis("equal")
    plt.grid(True)
    
    if show_plot: plt.show()

    return True


# メイン関数
if __name__ == "__main__":
    main()
