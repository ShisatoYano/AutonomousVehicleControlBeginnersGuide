"""
four_wheels_vehicle.py

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
from front_axle import FrontAxle
from rear_axle import RearAxle
from agent import Agent
from motion_model import MotionModel


class FourWheelsVehicle:
    """
    Four Wheels Vehicle model class
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
        self.steer_rad = 0.0
        self.spec = spec
        self.body = Body(spec)
        self.chassis = Chassis(spec)
        self.front_left_tire = Tire(spec, spec.f_len_m, spec.axle_half_m)
        self.front_right_tire = Tire(spec, spec.f_len_m, -spec.axle_half_m)
        self.rear_left_tire = Tire(spec, -spec.r_len_m, spec.axle_half_m)
        self.rear_right_tire = Tire(spec, -spec.r_len_m, -spec.axle_half_m)
        self.front_axle = FrontAxle(spec)
        self.rear_axle = RearAxle(spec)
        self.agent = agent
        self.motion = motion
        self.poses = [pose]

    def one_step(self, time_interval_s):
        if not self.agent: return
        order = self.agent.control_order()
        self.pose = self.motion.state_transition(self.pose, order, time_interval_s)
        self.steer_rad = self.motion.steering_angle_rad(order)
    
    def draw(self, axes, elems):
        elems += self.body.draw(axes, self.pose)
        elems += self.chassis.draw(axes, self.pose)
        elems += self.front_left_tire.draw(axes, self.pose, self.steer_rad)
        elems += self.front_right_tire.draw(axes, self.pose, self.steer_rad)
        elems += self.rear_left_tire.draw(axes, self.pose, 0.0)
        elems += self.rear_right_tire.draw(axes, self.pose, 0.0)
        elems += self.front_axle.draw(axes, self.pose)
        elems += self.rear_axle.draw(axes, self.pose)

        self.poses.append(self.pose)
        elems += axes.plot([p[0, 0] for p in self.poses], [p[1, 0] for p in self.poses], linewidth=0, marker=".", color=self.spec.color)


def main():
    vis = GlobalXYVisualizer(min_lim=0, max_lim=30, time_span_s=20)

    spec = VehicleSpecification()
    agent = Agent(2.0, 10.0/180*math.pi)
    motion = MotionModel(spec)
    vehicle = FourWheelsVehicle(np.array([[15.0], [0.0], [0.0]]),
                                spec, agent, motion)
    
    vis.add_object(vehicle)
    vis.draw()


if __name__ == "__main__":
    main()
