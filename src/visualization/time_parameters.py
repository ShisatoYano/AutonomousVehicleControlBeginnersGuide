"""
time_parameters.py

Author: Shisato Yano
"""

class TimeParameters:
    """
    Parameters class for visualization time setting
    """

    def __init__(self, span_sec=10, interval_sec=0.1):
        self.span_sec = span_sec
        self.interval_sec = interval_sec
        self.interval_msec = interval_sec * 1000
        self.frame_num = int(span_sec / interval_sec) + 1
    
    def get_interval_sec(self):
        return self.interval_sec
    
    def get_interval_msec(self):
        return self.interval_msec
    
    def get_frame_num(self):
        return self.frame_num
    
    def current_sec(self, index):
        return self.interval_sec * index
    
    def simulation_finished(self, index):
        return (self.interval_sec * index >= self.span_sec)
