"""
trapezoidal_speed_profile.py

Author: Shisato Yano
"""

class TrapezoidalSpeedProfile:
    """
    Speed profile class using trapezoidal input
    """

    def __init__(self, max_spd_mps, max_accel_mps2, distance_m):
        """
        Constructor
        max_spd_mps: Maximum speed[m/s]
        max_accel_mps2: Maximum acceleration/deceleration[m/s2]
        distance_m: Course distance[m]
        """
        
        self.max_spd_mps = max_spd_mps
        self.max_accel_mps2 = max_accel_mps2
        self.distance_m = distance_m

        self.accel_time_s = self.max_spd_mps / self.max_accel_mps2
        self.decel_time_s = self.accel_time_s

        accel_dist_m = self.max_spd_mps * self.accel_time_s / 2
        decel_dist_m = self.max_spd_mps * self.decel_time_s / 2
        self.const_time_s = (self.distance_m - accel_dist_m - decel_dist_m) / self.max_spd_mps
