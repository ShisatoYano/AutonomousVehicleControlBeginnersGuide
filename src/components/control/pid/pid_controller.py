"""
pid_controller.py

Author: Shisato Yano
"""

from math import sin, tan, atan2


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

        self.course = course
    
    def update(self, state):
        """
        Function to update data for path tracking
        state: Vehicle's state object
        """

        if not self.course: return
