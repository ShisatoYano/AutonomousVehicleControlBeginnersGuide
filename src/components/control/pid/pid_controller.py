"""
pid_controller.py

Author: Shisato Yano
"""


class PidController:
    """
    Controller class by PID control algorithm
    """

    def __init__(self, spec, course=None):
        """
        Constructor
        spec: Vehicle specification object
        course: Course data and logic object
        """

        self.WHEEL_BASE_M = spec.wheel_base_m
