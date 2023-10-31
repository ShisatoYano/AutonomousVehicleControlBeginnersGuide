"""
extended_kalman_filter_localizer.py

Author: Shisato Yano
"""

class ExtendedKalmanFilterLocalizer:
    """
    Self localization by Extended Kalman Filter class
    """

    def __init__(self, color='r'):
        """
        Constructor
        color: Color of drawing error covariance ellipse
        """

        self.color = color
