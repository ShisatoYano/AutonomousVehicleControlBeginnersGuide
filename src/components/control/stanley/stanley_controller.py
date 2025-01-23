"""
stanley_controller.py

Author: Shisato Yano
"""


class StanleyController:
    """
    Controller class by Stanley steering control algorithm
    """

    def __init__(self, spec, course=None):
        """
        Constructor
        course: Course data and logic object
        """

        self.SPEED_PROPORTIONAL_GAIN = 1.0
        self.CONTROL_GAIN = 0.5
        self.WHEEL_BASE_M = spec.wheel_base_m

        self.course = course
        self.target_course_index = 0
        self.target_accel_mps2 = 0.0
        self.target_speed_mps = 0.0
        self.target_yaw_rate_rps = 0.0
        self.target_steer_rad = 0.0
