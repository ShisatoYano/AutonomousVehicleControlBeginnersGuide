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

        self.data = [init_grid] * self.all_grids_num
    
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
    
    def calculate_xy_index_from_vector_index(self, vector_index):
        """
        Function to calculate x and y index from 1d vector index
        vector_index: index of 1d vector
        Return: x and y index in 2d array
        """
        
        y_idx, x_idx = divmod(vector_index, self.width_grids_num)
        return x_idx, y_idx

    def calculate_grid_center_pos_from_index(self, index, left_bottom_pos):
        """
        Function to calculate center position of a grid
        index: index of x or y axis
        left_bottom_pos: left bottom position of x or y axis
        Return: center position of x or y axis of a grid
        """
        
        return index * self.resolution_m + left_bottom_pos + self.resolution_m/2

    def calculate_grid_center_xy_pos_from_xy_index(self, x_idx, y_idx):
        """
        Function to calculate center x-y position of a grid from x-y 2d array index
        x_idx: index of grid on x axis
        y_idx: index of grid on y axis
        Return: center x-y position of a grid
        """
        
        center_x_pos = self.calculate_grid_center_pos_from_index(x_idx, self.left_bottom_x_m)
        center_y_pos = self.calculate_grid_center_pos_from_index(y_idx, self.left_bottom_y_m)
        return center_x_pos, center_y_pos

    def calculate_grid_center_xy_pos_from_vector_index(self, vector_index):
        """
        Function to calculate center x-y position of a grid from 1d vector index
        vector_index: index of 1d vector
        Return: center x-y position of a grid
        """
        
        x_idx, y_idx = self.calculate_xy_index_from_vector_index(vector_index)
        return self.calculate_grid_center_xy_pos_from_xy_index(x_idx, y_idx)
    
    def set_grid_data(self, index, grid):
        """
        Setter of grid data into map
        index: index of 1d vector
        grid: object of grid
        """
        
        self.data[index] = grid
    
    def get_grid_data(self, index):
        """
        Getter of grid data in map
        index: index of 1d vector
        Return: object of grid
        """
        
        return self.data[index]
