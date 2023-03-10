"""
two_wheels_vehicle.py

Author: Shisato Yano
"""

import matplotlib.pyplot as plt
import numpy as np
import math
import sys
import os

sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../visualization")
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../agent")
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../motion_model")
from global_xy_visualizer import GlobalXYVisualizer
from vehicle_specification import VehicleSpecification
from body import Body
from chassis import Chassis
from tire import Tire
from agent import Agent
from motion_model import MotionModel


class TwoWheelsVehicle:
    """
    Two Wheels Vehicle model class
    """

    def __init__(self, pose, spec, agent=None, motion=None):
        """
        Constructor
        pose: vehicle's pose [x[m], y[m], yaw[rad]]
        spec: vehicle's specification object
        agent: agent to decide control input
        motion: motion model for state transition
        """
        
        self.pose = pose
        self.spec = spec
        self.body = Body(spec)
        self.chassis = Chassis(spec)
        self.front_tire = Tire(spec, spec.f_len_m, 0.0)
        self.rear_tire = Tire(spec, -spec.r_len_m, 0.0)
        self.agent = agent
        self.motion = motion
        self.poses = [pose]

    def one_step(self, time_interval_s):
        if not self.agent or not self.motion: return
        order = self.agent.control_order()
        self.pose = self.motion.state_transition(self.pose, order, time_interval_s)
    
    def draw(self, axes, elems):
        x_m, y_m, yaw_rad = self.pose

        elems += self.body.draw(axes, self.pose)
        elems += self.chassis.draw(axes, self.pose)
        elems += self.front_tire.draw(axes, self.pose)
        elems += self.rear_tire.draw(axes, self.pose)

        self.poses.append(self.pose)
        elems += axes.plot([p[0, 0] for p in self.poses], [p[1, 0] for p in self.poses], linewidth=0, marker=".", color=self.spec.color)


def main():
    vis = GlobalXYVisualizer(min_lim=0, max_lim=30, time_span_s=20)

    spec = VehicleSpecification()
    agent = Agent(2.0, 10.0/180*math.pi)
    motion = MotionModel(spec)
    vehicle = TwoWheelsVehicle(np.array([[15.0], [0.0], [0.0]]),
                               spec, agent, motion)
    
    vis.add_object(vehicle)
    vis.draw()


if __name__ == "__main__":
    main()
