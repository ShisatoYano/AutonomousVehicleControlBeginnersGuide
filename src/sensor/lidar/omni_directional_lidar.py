"""
omni_directional_lidar.py

Author: Shisato Yano
"""

import numpy as np
import sys
from pathlib import Path

sys.path.append(str(Path(__file__).absolute().parent) + "/../../array")
from xy_array import XYArray

class OmniDirectionalLidar:
    def __init__(self, obst_list, inst_lon_m=0.0, inst_lat_m=0.0):
        self.obst_list = obst_list
        self.inst_pos_array = XYArray(np.array([[inst_lon_m], [inst_lat_m]]))
    
    def update(self, pose):
        transformed_array = self.inst_pos_array.homogeneous_transformation(pose[0, 0], pose[1, 0], pose[2, 0])
        self.inst_pos_array = transformed_array
    
    def draw(self, axes, elems):
        inst_pos_plot, = axes.plot(self.inst_pos_array.get_x_data(),
                                   self.inst_pos_array.get_y_data(),
                                   marker='.', color="pink")
        elems.append(inst_pos_plot)
