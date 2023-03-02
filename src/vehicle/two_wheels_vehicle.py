"""
two_wheels_vehicle.py

Author: Shisato Yano
"""

import matplotlib.pyplot as plt
import numpy as np
import sys
import os

sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../visualization")
from global_xy_visualizer import GlobalXYVisualizer
from vehicle_specification import VehicleSpecification
from body import Body
from chassis import Chassis
from tire import Tire


class TwoWheelsVehicle:
    """
    Two Wheels Vehicle model class
    """

    def __init__(self, pose, spec):
        """
        Constructor
        pose: vehicle's pose [x[m], y[m], yaw[rad]]
        spec: vehicle's specification object
        """
        
        self.pose = pose
        self.spec = spec
        self.body = Body(spec)
        self.chassis = Chassis(spec)
        self.front_tire = Tire(spec, spec.f_len_m, 0.0)
        self.rear_tire = Tire(spec, -spec.r_len_m, 0.0)

    
    def draw(self, axes, elems):
        x_m, y_m, yaw_rad = self.pose
        elems += self.body.draw_object(axes)
        elems += self.chassis.draw_object(axes)
        # self.front_tire.draw(axes)
        # self.rear_tire.draw(axes)


def main():
    vis = GlobalXYVisualizer()

    spec = VehicleSpecification()
    vehicle = TwoWheelsVehicle(np.array([[0.0], [0.0], [0.0]]), spec)
    
    vis.add_object(vehicle)
    vis.draw()


if __name__ == "__main__":
    main()
