"""
min_max.py

Author: Shisato Yano
"""

class MinMax:
    """
    Limitation values class of axis
    """

    def __init__(self, min=0, max=10):
        self.min = min
        self.max = max
    
    def min_value(self):
        return self.min
    
    def max_value(self):
        return self.max
