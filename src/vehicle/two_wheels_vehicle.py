"""
two_wheels_vehicle.py

Author: Shisato Yano
"""

import matplotlib.pyplot as plt
import numpy as np
import sys
import os

sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../visualization")
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../agent")
from global_xy_visualizer import GlobalXYVisualizer
from vehicle_specification import VehicleSpecification
from body import Body
from chassis import Chassis
from tire import Tire
from agent import Agent


class TwoWheelsVehicle:
    """
    Two Wheels Vehicle model class
    """

    def __init__(self, pose, spec, agent=None):
        """
        Constructor
        pose: vehicle's pose [x[m], y[m], yaw[rad]]
        spec: vehicle's specification object
        agent: agent to decide control input
        """
        
        self.pose = pose
        self.spec = spec
        self.body = Body(spec)
        self.chassis = Chassis(spec)
        self.front_tire = Tire(spec, spec.f_len_m, 0.0)
        self.rear_tire = Tire(spec, -spec.r_len_m, 0.0)
        self.agent = agent
        self.poses = [pose]

    def one_step(self, time_s):
        if not self.agent: return
        speed_msp, yaw_rate_rps = self.agent.control_input()
    
    def draw(self, axes, elems):
        x_m, y_m, yaw_rad = self.pose

        elems += self.body.draw_object(axes)
        elems += self.chassis.draw_object(axes)
        elems += self.front_tire.draw_object(axes)
        elems += self.rear_tire.draw_object(axes)

        self.poses.append(self.pose)
        elems += axes.plot([p[0] for p in self.poses], [p[1] for p in self.poses], linewidth=self.spec.line_w, color=self.spec.color)


def main():
    vis = GlobalXYVisualizer()

    spec = VehicleSpecification()
    vehicle = TwoWheelsVehicle(np.array([[0.0], [0.0], [0.0]]), spec)
    
    vis.add_object(vehicle)
    vis.draw()


if __name__ == "__main__":
    main()
