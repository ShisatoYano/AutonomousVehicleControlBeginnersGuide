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

        self.center_x_m = None # center x position of this grid
        self.center_y_m = None # center y position of this grid

        self.covariance = None # convariance matrix of points in this grid
        self.eigen_vectors = None # eigen vectors of covariance matrix
        self.eigen_values = None # eigen values of covariance matrix
