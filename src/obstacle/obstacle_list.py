"""
obstacle_list.py

Author: Shisato Yano
"""

class ObstacleList:
    def __init__(self):
        self.list = []
    
    def add_obstacle(self, obstacle):
        self.list.append(obstacle)
    
    def update(self, times_s):
        for obst in self.list: obst.update(times_s)
    
    def draw(self, axes, elems):
        for obst in self.list: obst.draw(axes, elems)
    
    def get_list(self):
        return self.list
