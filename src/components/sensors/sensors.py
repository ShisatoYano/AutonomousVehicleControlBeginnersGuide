"""
sensors.py

Author: Shisato Yano
"""

class Sensors:
    """
    Each sensors logic class
    """
    
    def __init__(self, lidar=None, gnss=None):
        """
        Constructor
        lidar: lidar object
        gnss: gnss object
        """

        self.lidar = lidar
        self.gnss = gnss
    
    def install(self, state):
        """
        Function to calculate each sensor's installation position on vehicle
        state: vehicle's state object
        """

        if self.lidar: self.lidar.install(state)

    def update_data(self, state):
        """
        Function to update each sensor's data
        state: vehicle's state object
        """

        if self.lidar: self.lidar.update(state)
        if self.gnss: self.gnss.update(state)
    
    def draw_data(self, axes, elems, state):
        """
        Function to draw each sensor's data
        axes: axes object of figure
        elems: list of plot object
        state: vehicle's state object
        """

        if self.lidar: self.lidar.draw(axes, elems, state)
        if self.gnss: self.gnss.draw(axes, elems)
    
    def get_point_cloud_from_lidar(self):
        """
        Function to get point cloud list from LiDAR
        If LiDAR was not installed, would return empty list
        """
        
        if self.lidar: return self.lidar.get_point_cloud()
        else: return []
    
    def get_lidar_global_x_m(self):
        """
        Function to get LiDAR's global position x[m]
        When LiDAR is not installed, return 0.0
        """
        
        if self.lidar: return self.lidar.get_global_x_m()
        else: return 0.0
    
    def get_lidar_global_y_m(self):
        """
        Function to get LiDAR's global position x[m]
        When LiDAR is not installed, return 0.0
        """

        if self.lidar: return self.lidar.get_global_y_m()
        else: return 0.0
    
    def get_xy_pos_from_gnss(self):
        """
        Function to get x-y position observed by GNSS
        If GNSS was not installed, return None
        """
        
        if self.gnss: return self.gnss.get_xy_pos()
        else: return None
