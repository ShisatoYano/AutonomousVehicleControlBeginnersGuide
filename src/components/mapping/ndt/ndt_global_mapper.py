"""
ndt_global_mapper.py

Author: Shisato Yano
"""

from ndt_map import NdtMap


class NdtGlobalMapper:
    """
    NDT global map construction class
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

        self.map = NdtMap(width_m, height_m, resolution_m, center_x_m, center_y_m, min_points_num)
    
    def update(self, point_cloud, state):
        """
        Function to update global map
        point_cloud: List of point objects scanned by LiDAR
        state: Vehicle's state to transform into global coordinate
        """
        
        vehicle_pose = state.x_y_yaw()

        points_xy_list = []
        for point in point_cloud:
            global_point_xy = point.get_transformed_data(vehicle_pose[0, 0],
                                                         vehicle_pose[1, 0],
                                                         vehicle_pose[2, 0])
            points_xy_list.append(global_point_xy)
        
        self.map.update_map(points_xy_list)
