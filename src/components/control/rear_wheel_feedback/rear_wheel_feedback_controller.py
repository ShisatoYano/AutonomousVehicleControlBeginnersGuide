"""
rear_wheel_feedback_controller.py

Author: Shisato Yano
"""


class RearWheelFeedbackController:
    """
    Controller class by Rear wheel feedback algorithm
    """

    def __init__(self, course=None):
        """
        Constructor
        course: Course data and logic object
        """

        self.course = course
