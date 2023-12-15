"""
ndt_grid.py

Author: Shisato Yano
"""


class NdtGrid:
    """
    NDT(Normal Distribution Transform) grid class
    """

    def __init__(self):
        """
        Constructor
        """
        
        self.points_num = 0 # number of points included in this grid

        self.mean_x_m = None # mean value of x position in this grid
        self.mean_y_m = None # mean value of y position in this grid

        self.covariance = None # convariance matrix of points in this grid
