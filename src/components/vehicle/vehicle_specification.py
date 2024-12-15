"""
vehicle_specification.py

Author: Shisato Yano
"""

import sys
from pathlib import Path

sys.path.append(str(Path(__file__).absolute().parent) + "/../visualization")
from min_max import MinMax

class VehicleSpecification:
    """
    Vehicle Specification parameters class
    """
    
    def __init__(self, f_len_m=2.0, r_len_m=0.0, tire_r_m=0.3,
                 tire_w_m=0.12, axle_half_m=0.5, color='k', 
                 line_w=1.0, line_type='-', area_size=10.0,
                 x_lim=MinMax(-30, 30), y_lim=MinMax(-30, 30),
                 max_accel_mps2=3.0):
        """
        Constructor
        f_len_m: length[m] from origin to center of front axle
        r_len_m: length[m] from origin to center of rear axle
        tire_r_m: tire's radius[m]
        tire_w_m: tire's half of width[m]
        color: vehicle's color
        line_w: plot line's width
        line_type: plot line's type
        area_size: plot area size[m]
        x_lim: min/max values of x-axis
        y_lim: min/max values of y-axis
        max_accel_mps2: maximum acceleration/deceleration[m/s2]
        """

        self.f_len_m = f_len_m
        self.f_edge_m = self.f_len_m + 0.5

        self.r_len_m = r_len_m
        self.r_edge_m = self.r_len_m + 0.5

        self.tread_m = 0.25 * (1.0 + self.f_len_m + self.r_len_m)
        self.width_m = 1.0 * self.tread_m
        self.wheel_base_m = self.f_len_m + self.r_len_m
        
        self.tire_r_m = tire_r_m
        self.tire_w_m = tire_w_m
        self.axle_half_m = axle_half_m

        self.color = color
        self.line_w = line_w
        self.line_type = line_type
        self.area_size = area_size

        self.x_lim = x_lim
        self.y_lim = y_lim

        self.max_accel_mps2 = max_accel_mps2
