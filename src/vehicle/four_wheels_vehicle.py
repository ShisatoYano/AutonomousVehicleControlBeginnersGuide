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
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../state")
from global_xy_visualizer import GlobalXYVisualizer
from vehicle_specification import VehicleSpecification
from body import Body
from chassis import Chassis
from front_left_tire import FrontLeftTire
from front_right_tire import FrontRightTire
from rear_left_tire import RearLeftTire
from rear_right_tire import RearRightTire
from front_axle import FrontAxle
from rear_axle import RearAxle
from state import State


class FourWheelsVehicle:
    """
    Four Wheels Vehicle model class
    """

    def __init__(self, state, spec):
        """
        Constructor
        state: vehicle's state object
        spec: vehicle's specification object
        """
        
        self.state = state
        self.spec = spec
        self.body = Body(spec)
        self.chassis = Chassis(spec)
        self.front_left_tire = FrontLeftTire(spec)
        self.front_right_tire = FrontRightTire(spec)
        self.rear_left_tire = RearLeftTire(spec)
        self.rear_right_tire = RearRightTire(spec)
        self.front_axle = FrontAxle(spec)
        self.rear_axle = RearAxle(spec)
        # self.poses = [pose]

    def one_step(self, time_s):
        updated_state = self.state.update(0.0, 0.17, time_s, self.spec.wheel_base_m)
        self.state = updated_state
    
    def draw(self, axes, elems):
        elems += self.body.draw(axes, self.state.x_y_yaw())
        elems += self.chassis.draw(axes, self.state.x_y_yaw())
        elems += self.front_left_tire.draw(axes, self.state.x_y_yaw(), 0.17)
        elems += self.front_right_tire.draw(axes, self.state.x_y_yaw(), 0.17)
        elems += self.rear_left_tire.draw(axes, self.state.x_y_yaw(), 0.0)
        elems += self.rear_right_tire.draw(axes, self.state.x_y_yaw(), 0.0)
        elems += self.front_axle.draw(axes, self.state.x_y_yaw())
        elems += self.rear_axle.draw(axes, self.state.x_y_yaw())

        # self.poses.append(self.pose)
        # elems += axes.plot([p[0, 0] for p in self.poses], [p[1, 0] for p in self.poses], linewidth=0, marker=".", color=self.spec.color)


def main():
    vis = GlobalXYVisualizer(min_lim=0, max_lim=30, time_span_s=20)

    spec = VehicleSpecification()
    state = State(15.0, 0.0, 0.0, 2.0)
    vehicle = FourWheelsVehicle(state, spec)
    
    vis.add_object(vehicle)
    vis.draw()


if __name__ == "__main__":
    main()
