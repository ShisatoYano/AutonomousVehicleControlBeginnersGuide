"""
binary_grid_mapper.py

Author: Shisato Yano
"""

from binary_map import BinaryMap


class BinaryGridMapper:
    """
    Binary occupancy grid map construction class
    """

    def __init__(self, width_m=60.0, height_m=60.0, resolution_m=1.0,
                 center_x_m=0.0, center_y_m=0.0, sensor_params=None):
        """
        Constructor
        width_m: Width size of map[m]
        height_m: Height size of map[m]
        resolution: Size of each cells[m]
        center_x_m: Center x position of map[m]
        center_y_m: Center y position of map[m]
        sensor_params: Parameters object of sensor
        """

        self.map = BinaryMap(width_m, height_m, resolution_m, center_x_m, center_y_m)
        self.params = sensor_params

    def update(self, point_cloud, state):
        """
        Function to update binary occupancy grid map
        point_cloud: List of points from LiDAR
        state: Vehicle's state to transform into global coordinate
        """

        vehicle_pose = state.x_y_yaw()

        points_x_list, points_y_list = [], []
        for point in point_cloud:
            global_x, global_y = point.get_transformed_data(self.params.INST_LON_M, self.params.INST_LAT_M, self.params.INST_YAW_RAD,
                                                            vehicle_pose[0, 0], vehicle_pose[1, 0], vehicle_pose[2, 0])
            points_x_list.append(global_x)
            points_y_list.append(global_y)
        
        self.map.update_map(points_x_list, points_y_list)
    
    def draw(self, axes, elems):
        """
        Function to draw map data
        axes: Axes object of figure
        elems: List of plot object
        """

        self.map.draw_map(axes, elems)
