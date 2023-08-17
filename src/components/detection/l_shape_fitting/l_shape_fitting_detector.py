"""
l_shape_fitting_detector.py

Author: Shisato Yano
"""

class LShapeFittingDetector:
    """
    Object detection class by L shape fitting algorithm
    """

    def __init__(self, min_rng_th_m=0.5, rng_th_rate=0.001):
        self.MIN_RNG_TH_M = min_rng_th_m
        self.RNG_TH_RATE = rng_th_rate
