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
    
    def update(self, state):
        """
        Function to update data for path tracking
        state: Vehicle's state object
        """

        if not self.course: return
    
    def draw(self, axes, elems):
        """
        Function to draw target point on course
        axes: Axes object of figure
        elems: plot object's list
        """

        pass
