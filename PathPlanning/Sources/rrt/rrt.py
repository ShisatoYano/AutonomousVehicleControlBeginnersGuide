"""
Path Planning sample program by
Rapidly-Exploring Random Trees.

Author: Shisato Yano
"""

import matplotlib.pyplot as plt
import math

# parameters
START_X_M = 10.0 # start point
START_Y_M = 10.0
GOAL_X_M = 50.0 # goal point
GOAL_Y_M = 50.0
MIN_SAMPLING_AREA_M = 0 # random sampling area
MAX_SAMPLING_AREA_M = 60
VEHICLE_SIZE_HALF_M = 2.0

# flag to switch showing output graph or not
show_plot = True

def main():
    print(__file__ + " start!!")

    # initialize plot
    ax = plt.subplot(1, 1, 1)
    ax.set_xlabel("X[m]")
    ax.set_ylabel("Y[m]")
    ax.set_aspect("equal")
    ax.grid(True)

    return True


# execute main process
if __name__ == "__main__":
    main()
