"""
sensors.py

Author: Shisato Yano
"""

class Sensors:
    def __init__(self, lidar=None, gnss=None):
        self.lidar = lidar
        self.gnss = gnss
    
    def install(self, state):
        if self.lidar: self.lidar.install(state)

    def update_data(self, state):
        if self.lidar: self.lidar.update(state)
    
    def draw_data(self, axes, elems, state):
        if self.lidar: self.lidar.draw(axes, elems, state)
