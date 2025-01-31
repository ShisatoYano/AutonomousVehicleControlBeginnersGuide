"""
binary_occupancy_grid.py

Author: Shantanu Parab
"""

import numpy as np
import sys
from pathlib import Path
from collections import defaultdict
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap


abs_dir_path = str(Path(__file__).absolute().parent)
relative_path = "/../../../components/"


sys.path.append(abs_dir_path + relative_path + "visualization")
sys.path.append(abs_dir_path + relative_path + "obstacle")
sys.path.append(abs_dir_path + relative_path + "state")


from min_max import MinMax
from obstacle_list import ObstacleList
from obstacle import Obstacle
from state import State
import json


# Define RGB colors for each grid value
# Colors in the format [R, G, B], where values are in the range [0, 1]
colors = [
    [1.0, 1.0, 1.0],  # Free space (white)
    [0.4, 0.8, 1.0],  # Explored nodes (light blue)
    [0.0, 1.0, 0.0],  # Path (green)
    [0.5, 0.5, 0.5],  # Clearance space (yellow-orange)
    [0.0, 0.0, 0.0],  # Obstacles (red)
]

# Create a colormap
custom_cmap = ListedColormap(colors)

class BinaryOccupancyGrid:
    def __init__(self, x_lim , y_lim, resolution, clearance, map_path):

        self.x_min, self.x_max = x_lim.min_value(), x_lim.max_value()
        self.y_min, self.y_max = y_lim.min_value(), y_lim.max_value()
        self.resolution = resolution
        self.clearance = clearance

        self.map, self.x_range, self.y_range =  self.create_grid()
        self.map_path = map_path


    def create_grid(self):
        """Create a grid based on the specified or derived limits."""

        x_range = np.arange(self.x_min, self.x_max, self.resolution)
        y_range = np.arange(self.y_min, self.y_max, self.resolution)

        map = np.zeros((len(y_range), len(x_range)))  # Initialize map as free space

        return map, x_range, y_range

    def add_object(self, obtacle_list: ObstacleList):
        """Mark obstacles and their clearance on the map, considering rotation (yaw)."""
        for obs in obtacle_list.list:
            # Get obstacle parameters
            x_c = obs.state.x_m
            y_c = obs.state.y_m
            yaw = obs.state.yaw_rad
            length = obs.length_m
            width = obs.width_m

            # Calculate the clearance dimensions
            clearance_length = length + self.clearance
            clearance_width = width + self.clearance

            # Define corners for the clearance area
            clearance_corners = np.array([
                [-clearance_length, -clearance_width],
                [-clearance_length, clearance_width],
                [clearance_length, clearance_width],
                [clearance_length, -clearance_width]
            ])

            # Define corners for the actual obstacle
            obstacle_corners = np.array([
                [-length, -width],
                [-length, width],
                [length, width],
                [length, -width]
            ])

            # Apply rotation to both obstacle and clearance corners
            rotation_matrix = np.array([
                [np.cos(yaw), -np.sin(yaw)],
                [np.sin(yaw), np.cos(yaw)]
            ])
            rotated_clearance_corners = np.dot(clearance_corners, rotation_matrix.T) + np.array([x_c, y_c])
            rotated_obstacle_corners = np.dot(obstacle_corners, rotation_matrix.T) + np.array([x_c, y_c])

            # Mark the clearance area
            self._mark_area(rotated_clearance_corners, value=0.75)  # 0.5 for clearance

            # Mark the actual obstacle area
            self._mark_area(rotated_obstacle_corners, value=1.0)  # 1.0 for obstacles

    def _point_in_polygon(self, x, y, corners):
        """
        Check if a point (x, y) is inside a polygon defined by corners.
        Args:
            x: X-coordinate of the point.
            y: Y-coordinate of the point.
            corners: Array of polygon corners in global coordinates.
        Returns:
            True if the point is inside the polygon, False otherwise.
        """
        n = len(corners)
        inside = False
        px, py = x, y
        for i in range(n):
            x1, y1 = corners[i]
            x2, y2 = corners[(i + 1) % n]
            if ((y1 > py) != (y2 > py)) and \
            (px < (x2 - x1) * (py - y1) / (y2 - y1 + 1e-6) + x1):
                inside = not inside
        return inside


    def _mark_area(self, corners, value):
        """
        Mark a rectangular area on the map based on the given rotated corners.
        Args:
            corners: The rotated corners of the area in global coordinates.
            value: The value to mark in the map (e.g., 0.5 for clearance, 1.0 for obstacles).
        """
        # Get the bounding box of the corners
        x_min = max(0, int((min(corners[:, 0]) - self.x_range[0]) / self.resolution))
        x_max = min(self.map.shape[1], int((max(corners[:, 0]) - self.x_range[0]) / self.resolution))
        y_min = max(0, int((min(corners[:, 1]) - self.y_range[0]) / self.resolution))
        y_max = min(self.map.shape[0], int((max(corners[:, 1]) - self.y_range[0]) / self.resolution))

        # Iterate through the map cells in the bounding box
        for x in range(x_min, x_max):
            for y in range(y_min, y_max):
                # Get the center of the current cell
                cell_x = self.x_range[0] + x * self.resolution + self.resolution / 2
                cell_y = self.y_range[0] + y * self.resolution + self.resolution / 2

                # Check if the cell center is inside the rotated polygon
                if self._point_in_polygon(cell_x, cell_y, corners):
                    self.map[y, x] = max(self.map[y, x], value)  # Mark the cell

    # Save the map to a file as an image/json
    def save_map(self):
        """
        Save the map to a file.
        """

        if self.map_path.endswith('.npy'):
            np.save(self.map_path, self.map)
        elif self.map_path.endswith('.png'):
            plt.imsave(self.map_path, self.map, cmap=custom_cmap, origin='lower')
        elif self.map_path.endswith('.json'):
            map_list = self.map.tolist()
            with open(self.map_path, 'w') as f:
                json.dump(map_list, f)
        else:
            raise ValueError("Unsupported file format. Use .npy, .png, or .json")



if  __name__ == "__main__":

    obst_list = ObstacleList()
    obst_list.add_obstacle(Obstacle(State(x_m=10.0, y_m=15.0), length_m=10, width_m=8))
    obst_list.add_obstacle(Obstacle(State(x_m=40.0, y_m=0.0), length_m=2, width_m=10))
    obst_list.add_obstacle(Obstacle(State(x_m=10.0, y_m=-10.0, yaw_rad=np.rad2deg(45)), length_m=5, width_m=5))
    obst_list.add_obstacle(Obstacle(State(x_m=30.0, y_m=15.0, yaw_rad=np.rad2deg(10)), length_m=5, width_m=2))

    bin_occ_grid = BinaryOccupancyGrid(MinMax(-5, 55), MinMax(-20, 25), 0.5, 1.5)
    bin_occ_grid.add_object(obst_list)

    bin_occ_grid.save_map("map.json")

    plt.figure(figsize=(10, 8))
    plt.imshow(bin_occ_grid.map, extent=[bin_occ_grid.x_range[0], bin_occ_grid.x_range[-1], bin_occ_grid.y_range[0], bin_occ_grid.y_range[-1]],
            origin='lower', cmap=custom_cmap)

    plt.legend()
    plt.show()
