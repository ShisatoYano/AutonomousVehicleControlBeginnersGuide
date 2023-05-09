"""
omni_directional_lidar.py

Author: Shisato Yano
"""

import numpy as np
import matplotlib.patches as ptc
import sys
from pathlib import Path
from math import atan2, sin, cos

sys.path.append(str(Path(__file__).absolute().parent) + "/../../array")
from xy_array import XYArray
from scan_point import ScanPoint

class OmniDirectionalLidar:
    def __init__(self, obst_list, inst_lon_m=0.0, inst_lat_m=0.0, 
                 min_range_m=0.5, max_range_m=20, resolution_rad=np.deg2rad(1.0)):
        self.obst_list = obst_list
        self.inst_lon_list = [inst_lon_m]
        self.inst_lat_list = [inst_lat_m]
        self.inst_pos_array = XYArray(np.array([self.inst_lon_list, self.inst_lat_list]))
        self.MIN_RANGE_M = min_range_m
        self.MAX_RANGE_M = max_range_m
        self.RESOLUTION_RAD = resolution_rad
        self.DIST_DB_SIZE = int(np.floor((np.pi * 2.0) / self.RESOLUTION_RAD)) + 1
        self.MAX_DB_VALUE = float("inf")
        self.DELTA_LIST = np.arange(0.0, 1.0, 0.008)
        self.latest_point_cloud = []
    
    def _visible(self, distance_m):
        return (self.MIN_RANGE_M <= distance_m <= self.MAX_RANGE_M)
    
    def _normalize_angle_until_2pi(self, angle_rad):
        if 0.0 > angle_rad: return (angle_rad + np.pi * 2.0)
        else: return angle_rad
    
    def _normalize_angle_pi_2_pi(self, angle_rad):
        if angle_rad > np.pi: return (angle_rad - np.pi * 2.0)
        else: return angle_rad
    
    def _ray_casting_filter(self, distance_list, angle_list):
        point_cloud = []
        dist_db = [self.MAX_DB_VALUE for _ in range(self.DIST_DB_SIZE)]

        for i in range(len(angle_list)):
            normalized_angle_2pi = self._normalize_angle_until_2pi(angle_list[i])
            angle_id = int(round(normalized_angle_2pi / self.RESOLUTION_RAD)) % self.DIST_DB_SIZE
            if dist_db[angle_id] > distance_list[i]:
                dist_db[angle_id] = distance_list[i]
        
        for i in range(len(dist_db)):
            angle_rad = i * self.RESOLUTION_RAD
            normalized_angle_pi_2_pi = self._normalize_angle_pi_2_pi(angle_rad)
            distance_m = dist_db[i]
            if (distance_m != self.MAX_DB_VALUE) and self._visible(distance_m):
                x_m = distance_m * cos(normalized_angle_pi_2_pi)
                y_m = distance_m * sin(normalized_angle_pi_2_pi)
                point_cloud.append(ScanPoint(distance_m, normalized_angle_pi_2_pi, x_m, y_m))
        
        self.latest_point_cloud = point_cloud
    
    def _interpolate(self, x_1, x_2, delta):
        return ((1.0 - delta) * x_1 + delta * x_2)

    def _calculate_contour_xy(self, vertex_x, vertex_y):
        contour_x, contour_y = [], []
        len_vertex = len(vertex_x)

        for i in range(len_vertex - 1):
            contour_x.extend([self._interpolate(vertex_x[i], vertex_x[i+1], delta) for delta in self.DELTA_LIST])
            contour_y.extend([self._interpolate(vertex_y[i], vertex_y[i+1], delta) for delta in self.DELTA_LIST])
        
        contour_x.extend([self._interpolate(vertex_x[len_vertex-1], vertex_x[1], delta) for delta in self.DELTA_LIST])
        contour_y.extend([self._interpolate(vertex_y[len_vertex-1], vertex_y[1], delta) for delta in self.DELTA_LIST])

        return contour_x, contour_y

    def update(self, pose):
        transformed_array = self.inst_pos_array.homogeneous_transformation(pose[0, 0], pose[1, 0], pose[2, 0])
        self.inst_lon_list = transformed_array.get_x_data()
        self.inst_lat_list = transformed_array.get_y_data()

        distance_list, angle_list = [], []
        for obst in self.obst_list.get_list():
            vertex_x, vertex_y = obst.vertex_xy()
            contour_x, contour_y = self._calculate_contour_xy(vertex_x, vertex_y)
            for x, y in zip(contour_x, contour_y):
                diff_x = x - self.inst_lon_list[0]
                diff_y = y - self.inst_lat_list[0]
                distance_m = np.hypot(diff_x, diff_y)
                angle_rad = atan2(diff_y, diff_x) - pose[2, 0]
                distance_list.append(distance_m)
                angle_list.append(angle_rad)
        
        self._ray_casting_filter(distance_list, angle_list)
    
    def draw(self, axes, pose, elems):
        inst_pos_plot, = axes.plot(self.inst_lon_list, self.inst_lat_list, marker='.', color='b')
        elems.append(inst_pos_plot)

        for point in self.latest_point_cloud:
            point.draw(axes, self.inst_lon_list[0], self.inst_lat_list[0], pose[2, 0], elems)
