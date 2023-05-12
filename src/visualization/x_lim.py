"""
x_lim.py

Author: Shisato Yano
"""

class XLim:
    """
    Limitation values class of x axis
    """

    def __init__(self, min=0, max=10):
        self.min = min
        self.max = max
    
    def min_value(self):
        return self.min
    
    def max_value(self):
        return self.max
