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
        self.target_course_index = 0
        self.target_accel_mps2 = 0.0
        self.target_yaw_rate_rps = 0.0
        self.target_steer_rad = 0.0
    
    def _calculate_target_course_index(self, state):
        """
        Private function to calculate target point's index on course
        state: Vehicle's state object
        """
        
        nearest_index = self.course.search_nearest_point_index(state)
        self.target_course_index = nearest_index

    def update(self, state):
        """
        Function to update data for path tracking
        state: Vehicle's state object
        """

        if not self.course: return

        self._calculate_target_course_index(state)
    
    def get_target_accel_mps2(self):
        """
        Function to get acceleration input[m/s2]
        """
        
        return self.target_accel_mps2

    def get_target_yaw_rate_rps(self):
        """
        Function to get yaw rate input[rad/s]
        """

        return self.target_yaw_rate_rps

    def get_target_steer_rad(self):
        """
        Function to get steering angle input[rad]
        """
        
        return self.target_steer_rad

    def draw(self, axes, elems):
        """
        Function to draw target point on course
        axes: Axes object of figure
        elems: plot object's list
        """

        pass
