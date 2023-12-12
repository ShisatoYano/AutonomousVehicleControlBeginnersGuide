"""
grid_map.py

Author: Shisato Yano
"""

import numpy as np

from float_grid import FloatGrid


class GridMap:
    """
    Grid map class
    """

    def __init__(self, width_m=60.0, height_m=60.0, resolution_m=5.0,
                 center_x_m=0.0, center_y_m=0.0, init_grid=FloatGrid(0.0)):
        """
        Constructor
        width_m: Width size of map[m]
        height_m: Height size of map[m]
        resolution: Size of each grids[m]
        center_x_m: Center x position of map[m]
        center_y_m: Center y position of map[m]
        init_grid: Initial grid object
        """

        self.width_m = width_m
        self.height_m = height_m
        self.resolution_m = resolution_m
        self.center_x_m = center_x_m
        self.center_y_m = center_y_m

        self.width_grids_num = int(np.floor(self.width_m / self.resolution_m))
        self.height_grids_num = int(np.floor(self.height_m / self.resolution_m))
        self.all_grids_num = self.width_grids_num * self.height_grids_num

        self.left_bottom_x_m = self.center_x_m - self.width_m / 2.0
        self.left_bottom_y_m = self.center_y_m - self.height_m / 2.0

        self.map = [init_grid] * self.all_grids_num
    
    def calculate_xy_index_from_position(self, pos, left_bottom_pos, max_index):
        """
        Function to calculate x/y index of position in 2d array
        pos: x or y position
        left_bottom_pos: left bottom position of map(origin of map)
        max_index: maximum number of x/y index
        Return: index of 2d array
        """
        
        index = int(np.floor((pos - left_bottom_pos) / self.resolution_m))
        if 0 <= index <= max_index: return index
        else: return None

    def calculate_vector_index_from_xy_index(self, x_idx, y_idx):
        """
        Function to calculate index of x-y index in 1d vector
        x_idx: index of x axis
        y_idx: index of y axis
        Return: index of 1d vector
        """
        
        vector_index = int(y_idx * self.width_grids_num + x_idx)
        return vector_index

    def calculate_vector_index_from_position(self, x_m, y_m):
        """
        Function to calculate index of x-y point in 1d vector
        x_m: x coordinate of position
        y_m: y coordinate of position
        Return: index of 1d vector
        """

        x_idx = self.calculate_xy_index_from_position(x_m, self.left_bottom_x_m, self.width_grids_num)
        y_idx = self.calculate_xy_index_from_position(y_m, self.left_bottom_y_m, self.height_grids_num)
        return self.calculate_vector_index_from_xy_index(x_idx, y_idx)

