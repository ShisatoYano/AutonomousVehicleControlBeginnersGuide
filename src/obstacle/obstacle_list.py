"""
obstacle_list.py

Author: Shisato Yano
"""

class ObstacleList:
    """
    Obstacles list class
    """
    
    def __init__(self):
        """
        Constructor
        """
        
        self.list = []
    
    def add_obstacle(self, obstacle):
        """
        Function to add obstacle object into list
        obstacle: Obstacle class's object
        """
        
        self.list.append(obstacle)
    
    def update(self, times_s):
        """
        Function to update each obstacle's state in list
        time_s: Time interval value[sec]
        """
        
        for obst in self.list: obst.update(times_s)
    
    def draw(self, axes, elems):
        """
        Function to draw each obstacles in list
        axes: Axes object of figure
        elems: List of plot objects
        """
        
        for obst in self.list: obst.draw(axes, elems)
    
    def get_list(self):
        """
        Function to get obstacle's list
        """
        
        return self.list
