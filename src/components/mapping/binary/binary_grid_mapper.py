"""
binary_grid_mapper.py

Author: Shisato Yano
"""


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

        self.params = sensor_params
