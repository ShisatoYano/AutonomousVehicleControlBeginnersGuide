"""
float_grid.py

Author: Shisato Yano
"""


class FloatGrid:
    """
    Class of grid which has float value data
    """

    def __init__(self, value=0.0):
        """
        Constructor
        value: initial value of data
        """
        
        self.data = value
    
    def get_data(self):
        """
        Getter of data
        Return: value of data
        """
        
        return self.data
