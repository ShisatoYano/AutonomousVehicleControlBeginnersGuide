"""
state.py

Author: Shisato Yano
"""


class State:
    """
    Vehicle state(x, y, yaw, speed) data and logic class
    """

    def __init__(self, x_m, y_m, yaw_rad, speed_mps):
        """
        Constructor
        x_m: Vehicle's position, x[m]
        y_m: Vehicle's position, y[m]
        yaw_rad: Vehicle's yaw angle[rad]
        speed_mps: Vehicle's speed[m/s]
        """

        self.x_m = x_m
        self.y_m = y_m
        self.yaw_rad = yaw_rad
        self.speed_mps = speed_mps
