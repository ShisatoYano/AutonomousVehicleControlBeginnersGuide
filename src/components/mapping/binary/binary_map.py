"""
binary_map.py

Author: Shisato Yano
"""

import numpy as np
import sys
from pathlib import Path
sys.path.append(str(Path(__file__).absolute().parent) + "/../grid")
from grid_map import GridMap
from grid_map import FloatGrid

class BinaryMap:
    """
    Binary grid map class
    """

    def __init__(self, width_m=60.0, height_m=60.0, resolution_m=1.0,
                 center_x_m=0.0, center_y_m=0.0):
        """
        Constructor
        width_m: Width size of map[m]
        height_m: Height size of map[m]
        resolution_m: Size of each cells[m]
        center_x_m: Center x position of map[m]
        center_y_m: Center y position of map[m]
        """
        
        self.map = GridMap(width_m, height_m, resolution_m,
                           center_x_m, center_y_m)

    def update_map(self, points_x_list, points_y_list):
        """
        Function to update map
        points_x_list: List of x coordinates of point cloud
        points_y_list: List of y coordinates of point cloud
        """

        points_x_array, points_y_array = np.array(points_x_list), np.array(points_y_list)
        
        for i in range(len(points_x_array)):
            index = self.map.calculate_vector_index_from_position(points_x_array[i],
                                                                  points_y_array[i])
            if index is not None:
                self.map.set_grid_data(index, FloatGrid(value=1.0))

    def draw_map(self, axes, elems):
        """"
        Function to draw map data
        axes: Axes object of figure
        elems: List of plot object
        """

        pass
