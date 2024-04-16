"""
ndt_map.py

Author: Shisato Yano
"""

import numpy as np
import sys
from pathlib import Path
from collections import defaultdict

sys.path.append(str(Path(__file__).absolute().parent) + "/../grid")
sys.path.append(str(Path(__file__).absolute().parent) + "/../../common")
from grid_map import GridMap
from ndt_grid import NdtGrid
from plot_lib import draw_covariance_ellipse


class NdtMap:
    """
    NDT grid map class
    """

    def __init__(self, width_m=60.0, height_m=60.0, resolution_m=5.0,
                 center_x_m=0.0, center_y_m=0.0, min_points_num=3):
        """
        Constructor
        width_m: Width size of map[m]
        height_m: Height size of map[m]
        resolution: Size of each grids[m]
        center_x_m: Center x position of map[m]
        center_y_m: Center y position of map[m]
        min_points_num: Minimum number of points for normal distribution transform
        """

        self.map = GridMap(width_m, height_m, resolution_m, 
                           center_x_m, center_y_m, init_grid=NdtGrid())
        
        self.MIN_POINTS_NUM = min_points_num
    
    def _create_grid_points_index_map(self, points_x_array, points_y_array):
        """
        Function to create map between grid index and points index
        points_x_array: ndarray of points x coordinate
        points_y_array: ndarray of points y coordinate
        Return: Map between grid index and points index
        """
        
        grid_points_index_map = defaultdict(list)

        for i in range(len(points_x_array)):
            index = self.map.calculate_vector_index_from_position(points_x_array[i],
                                                                  points_y_array[i])
            grid_points_index_map[index].append(i)
        
        return grid_points_index_map

    def update_map(self, points_x_list, points_y_list):
        """
        Function to update map
        points_x_list: List of point cloud x coordinates
        points_y_list: List of point cloud y coordinates
        """
        
        points_x_array, points_y_array = np.array(points_x_list), np.array(points_y_list)
        grid_points_index_map = self._create_grid_points_index_map(points_x_array, points_y_array)
        for grid_idx, points_indices in grid_points_index_map.items():
            new_points_num = len(points_indices)
            last_points_num = self.map.get_grid_data(grid_idx).points_num
            if new_points_num >= self.MIN_POINTS_NUM and new_points_num >= last_points_num:
                grid = NdtGrid()
                grid.points_num = len(points_indices)
                grid.mean_x_m = points_x_array[points_indices].mean()
                grid.mean_y_m = points_y_array[points_indices].mean()
                grid.covariance = np.cov(points_x_array[points_indices],
                                         points_y_array[points_indices])
                self.map.set_grid_data(grid_idx, grid)

    def draw_map(self, axes, elems):
        """
        Function to draw map data
        axes: Axes object of figure
        elems: List of plot object
        """

        [draw_covariance_ellipse(axes, elems, grid.mean_x_m, grid.mean_y_m, grid.covariance) for grid in self.map.data if grid.points_num > 0]
