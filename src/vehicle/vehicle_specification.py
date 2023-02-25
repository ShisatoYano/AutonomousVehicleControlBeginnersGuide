"""
vehicle_specification.py

Author: Shisato Yano
"""

class VehicleSpecification:
    def __init__(self, f_len_m=2.0, r_len_m=0.0, tire_r_m=0.3,
                 tire_w_m=0.12, color='k', line_w=1.0, line_type='-'):
        self.f_len_m = f_len_m
        self.r_len_m = r_len_m
        
        self.tire_r_m = tire_r_m
        self.tire_w_m = tire_w_m

        self.color = color
        self.line_w = line_w
        self.line_type = line_type
