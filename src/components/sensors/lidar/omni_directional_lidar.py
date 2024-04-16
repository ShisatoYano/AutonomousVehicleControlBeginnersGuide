"""
omni_directional_lidar.py

Author: Shisato Yano
"""

import numpy as np
from math import atan2, sin, cos
from scipy.stats import norm

from scan_point import ScanPoint

class OmniDirectionalLidar:
    """
    Sensing simulation class with Omni directional LiDAR
    """
    
    def __init__(self, obst_list, params):
        """
        Constructor
        obst_list: List of Obstacle objects
        params: Sensor parameters object
        """
        
        self.obst_list = obst_list
        self.params = params
        self.DIST_DB_SIZE = int(np.floor((np.pi * 2.0) / self.params.RESO_RAD)) + 1
        self.MAX_DB_VALUE = float("inf")
        self.DELTA_LIST = np.arange(0.0, 1.0, 0.008)
        self.latest_point_cloud = []
    
    def install(self, state):
        """
        Function to calculate installed position on global coordinate
        state: Vehicle's state object 
        """
        
        self.params.calculate_global_pos(state)

    def _visible(self, distance_m):
        """
        Private function to check object is visible according to sensing distance
        distance_m: Sensing distance[m]
        """
        
        return (self.params.MIN_RANGE_M <= distance_m <= self.params.MAX_RANGE_M)
    
    def _normalize_angle_until_2pi(self, angle_rad):
        """
        Private function to normalize sensing angle between 0 and 360 deg
        angle_rad: Sensing angle[rad]
        """

        if 0.0 > angle_rad: return (angle_rad + np.pi * 2.0)
        else: return angle_rad
    
    def _normalize_angle_pi_2_pi(self, angle_rad):
        """
        Private function to normalize sensing angle between -180 and 180 deg
        angle_rad: Sensing angle[rad]
        """

        if angle_rad > np.pi: return (angle_rad - np.pi * 2.0)
        else: return angle_rad
    
    def _ray_casting_filter(self, distance_list, angle_list, state):
        """
        Private function to filter point cloud by Ray casting
        distance_list: List of sensing distance[m]
        angle_list: List of sensing angle[rad]
        """

        point_cloud = []
        dist_db = [self.MAX_DB_VALUE for _ in range(self.DIST_DB_SIZE)]

        for i in range(len(angle_list)):
            normalized_angle_2pi = self._normalize_angle_until_2pi(angle_list[i])
            angle_id = int(round(normalized_angle_2pi / self.params.RESO_RAD)) % self.DIST_DB_SIZE
            if dist_db[angle_id] > distance_list[i]:
                dist_db[angle_id] = distance_list[i]
        
        for i in range(len(dist_db)):
            angle_rad = i * self.params.RESO_RAD
            angle_pi_2_pi = self._normalize_angle_pi_2_pi(angle_rad)
            distance_m = dist_db[i]
            if (distance_m != self.MAX_DB_VALUE) and self._visible(distance_m):
                angle_with_noise = norm.rvs(loc=angle_pi_2_pi, scale=self.params.ANGLE_STD_SCALE)
                dist_with_noise = norm.rvs(loc=distance_m, scale=self.params.DIST_STD_RATE*distance_m)
                x_m = dist_with_noise * cos(angle_with_noise)
                y_m = dist_with_noise * sin(angle_with_noise)
                point = ScanPoint(dist_with_noise, angle_with_noise, x_m, y_m)
                vehicle_pose = state.x_y_yaw()
                point.calculate_transformed_point(self.params.INST_LON_M, self.params.INST_LAT_M, self.params.INST_YAW_RAD,
                                                  vehicle_pose[0, 0], vehicle_pose[1, 0], vehicle_pose[2, 0])
                point_cloud.append(point)
        
        self.latest_point_cloud = point_cloud
    
    def _interpolate(self, x_1, x_2, delta):
        """
        Private function to interpolate between two values
        x_1: value 1
        x_2: value 2
        delta: resolution between value 1 and 2
        """

        return ((1.0 - delta) * x_1 + delta * x_2)

    def _calculate_contour_xy(self, vertex_x, vertex_y):
        """
        Private function to calculate contour coordinates x-y
        vertex_x: List of vertex's x coordinate
        vertex_y: List of vertex's y coordinate
        """

        contour_x, contour_y = [], []
        len_vertex = len(vertex_x)

        for i in range(len_vertex - 1):
            contour_x.extend([self._interpolate(vertex_x[i], vertex_x[i+1], delta) for delta in self.DELTA_LIST])
            contour_y.extend([self._interpolate(vertex_y[i], vertex_y[i+1], delta) for delta in self.DELTA_LIST])
        
        contour_x.extend([self._interpolate(vertex_x[len_vertex-1], vertex_x[1], delta) for delta in self.DELTA_LIST])
        contour_y.extend([self._interpolate(vertex_y[len_vertex-1], vertex_y[1], delta) for delta in self.DELTA_LIST])

        return contour_x, contour_y

    def update(self, state):
        """
        Function to update sensed point cloud data
        state: Vehicle's state
        """
        
        self.params.calculate_global_pos(state)

        self.params.calculate_sensor_odometry(state)

        self.params.calibrate_extrinsic_params(state)

        distance_list, angle_list = [], []
        for obst in self.obst_list.get_list():
            vertex_x, vertex_y = obst.vertex_xy()
            contour_x, contour_y = self._calculate_contour_xy(vertex_x, vertex_y)
            for x, y in zip(contour_x, contour_y):
                diff_x = x - self.params.get_global_x_m()
                diff_y = y - self.params.get_global_y_m()
                distance_m = np.hypot(diff_x, diff_y)
                angle_rad = atan2(diff_y, diff_x) - state.get_yaw_rad()
                distance_list.append(distance_m)
                angle_list.append(angle_rad)
        
        self._ray_casting_filter(distance_list, angle_list, state)
    
    def draw(self, axes, elems, state):
        """
        Function to draw sensed point cloud data
        axes: Axes object of figure
        elems: List of plot objects
        state: Vehicle's state object
        """

        self.params.draw_pos(axes, elems, state)

        for point in self.latest_point_cloud:
            point.draw(axes, elems)

    def get_point_cloud(self):
        """
        Function to get latest point cloud data
        Each points are calculated on sensor coordinate system
        """

        return self.latest_point_cloud                

    def get_global_x_m(self):
        """
        Function to get installation position x in global coordinate[m]
        """
        
        return self.params.get_global_x_m()
    
    def get_global_y_m(self):
        """
        Function to get installation position y in global coordinate[m]
        """

        return self.params.get_global_y_m()
