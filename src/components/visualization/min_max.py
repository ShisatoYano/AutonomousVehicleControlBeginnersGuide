"""
min_max.py

Author: Shisato Yano
"""

class MinMax:
    """
    Limitation values class of axis
    """

    def __init__(self, min=0, max=10):
        """
        Constructor
        min: Minimum value of axis
        max: Maximum value of axis
        """

        self.min = min
        self.max = max
    
    def min_value(self):
        """
        Function to get minimum value
        """
        
        return self.min
    
    def max_value(self):
        """
        Function to get maximum value
        """
        
        return self.max
