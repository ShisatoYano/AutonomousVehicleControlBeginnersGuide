"""
scan_point.py

Author: Shisato Yano
"""

import numpy as np
import sys
from pathlib import Path

sys.path.append(str(Path(__file__).absolute().parent) + "/../../array")
sys.path.append(str(Path(__file__).absolute().parent) + "/../../common")
from xy_array import XYArray
from matrix_lib import hom_mat_33


class ScanPoint:
    """
    Scan point of sensor class includes each sensing data
    """
    
    def __init__(self, distance_m, angle_rad, x_m, y_m):
        """
        Constructor
        distance_m: sensed distance data[m]
        angle_rad: sensed angle data[rad]
        x_m: sensed point's x coordinate data[m]
        y_m: sensed point's y coordinate data[m]
        """
        
        self.distance_m = distance_m
        self.angle_rad = angle_rad
        self.point_array = XYArray(np.array([[x_m], [y_m]]))
        self.transformed_x = None
        self.transformed_y = None
    
    def get_dimension(self):
        """
        Return point's x-y array data's dimension value
        """

        return self.point_array.get_dimension()
    
    def get_distance_m(self):
        """
        Return point's distance data[m]
        """
        
        return self.distance_m
    
    def get_point_array(self):
        """
        Return point's x-y array data
        Type is ndarray object
        """

        return self.point_array.get_data()
    
    def get_transformed_data(self, sensor_lon, sensor_lat, sensor_yaw,
                             vehicle_x, vehicle_y, vehicle_yaw):
        """
        Return transformed x-y array data based on specific coordinate system
        Type is ndarray object
        sensor_lon: longitudinal position of sensor on vehicle coordinate[m]
        sensor_lat: lateral position of sensor on vehicle coordinate[m]
        sensor_yaw: yaw angle of sensor on vehicle coordinate[rad]
        vehicle_x: x position of vehicle on global coordinate[m]
        vehicle_y: y position of vehicle on global coordinate[m]
        vehicle_yaw: yaw angle of vehicle on global coordinate[rad]
        """

        # transformation matrix on sensor coordinate
        point_xy = self.point_array.get_data()
        sensor_tf = hom_mat_33(point_xy[0, 0], point_xy[1, 0], 0.0)

        # transformation matrix on vehicle coordinate
        vehicle_tf = hom_mat_33(sensor_lon, sensor_lat, sensor_yaw)

        # transformation matrix on global coordinate
        global_tf = hom_mat_33(vehicle_x, vehicle_y, vehicle_yaw)

        # homegeneous transformation from sensor to global coordinate
        transformed_points_matrix = global_tf @ vehicle_tf @ sensor_tf

        return transformed_points_matrix[0, 2], transformed_points_matrix[1, 2]
    
    def calculate_transformed_point(self, sensor_lon, sensor_lat, sensor_yaw,
                                    vehicle_x, vehicle_y, vehicle_yaw):
        """
        Function to calculate transformed x-y point based on specific coordinate system
        sensor_lon: longitudinal position of sensor on vehicle coordinate[m]
        sensor_lat: lateral position of sensor on vehicle coordinate[m]
        sensor_yaw: yaw angle of sensor on vehicle coordinate[rad]
        vehicle_x: x position of vehicle on global coordinate[m]
        vehicle_y: y position of vehicle on global coordinate[m]
        vehicle_yaw: yaw angle of vehicle on global coordinate[rad]
        """
        
        self.transformed_x, self.transformed_y = \
            self.get_transformed_data(sensor_lon, sensor_lat, sensor_yaw,
                                      vehicle_x, vehicle_y, vehicle_yaw)

    def draw(self, axes, elems,
             sensor_lon, sensor_lat, sensor_yaw,
             vehicle_x, vehicle_y, vehicle_yaw):
        """
        Function to draw scan point's x-y coordinate data
        axes: Axes object of figure
        elems: List of plot objects
        sensor_lon: longitudinal position of sensor on vehicle coordinate[m]
        sensor_lat: lateral position of sensor on vehicle coordinate[m]
        sensor_yaw: yaw angle of sensor on vehicle coordinate[rad]
        vehicle_x: x position of vehicle on global coordinate[m]
        vehicle_y: y position of vehicle on global coordinate[m]
        vehicle_yaw: yaw angle of vehicle on global coordinate[rad]
        """

        # transformed_x, transformed_y = self.get_transformed_data(sensor_lon, sensor_lat, sensor_yaw,
        #                                                          vehicle_x, vehicle_y, vehicle_yaw)
        if self.transformed_x and self.transformed_y:
            point_plot, = axes.plot(self.transformed_x, self.transformed_y, marker='.', color='b')
            elems.append(point_plot)
