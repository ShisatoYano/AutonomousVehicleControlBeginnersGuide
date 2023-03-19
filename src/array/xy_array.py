"""
xy_array.py

Author: Shisato Yano
"""


class XYArray:
    """
    X-Y 2D array data and logic class
    """

    def __init__(self, array):
        """
        Constructor
        array: np.array([[x1, x2,..., xn], [y1, y2,..., yn]])
        """

        self.array = array
