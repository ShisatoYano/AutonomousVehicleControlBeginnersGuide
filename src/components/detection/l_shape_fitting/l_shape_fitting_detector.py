"""
l_shape_fitting_detector.py

Author: Shisato Yano
"""

class LShapeFittingDetector:
    """
    Object detection class by L shape fitting algorithm
    """

    def __init__(self, min_range_threshold_m=0.5, range_thresh_rate=0.001):
        self.MIN_RANGE_THRESHOLD_M = min_range_threshold_m
        self.RANGE_THRESH_RATE = range_thresh_rate
