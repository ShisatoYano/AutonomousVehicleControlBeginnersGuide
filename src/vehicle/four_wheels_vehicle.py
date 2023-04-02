"""
four_wheels_vehicle.py

Author: Shisato Yano
"""

import numpy as np
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
from state_history import StateHistory


class FourWheelsVehicle:
    """
    Four Wheels Vehicle model class
    """

    def __init__(self, state, history, spec, draw_area_width=10.0,
                 trajectory=None, controller=None):
        """
        Constructor
        state: Vehicle's state object
        history: Vehicle's state history object
        spec: Vehicle's specification object
        draw_area_width: Drawing area's width around Vehicle
        """
        
        self.state = state
        self.history = history

        self.spec = spec
        self.body = Body(spec)
        self.chassis = Chassis(spec)
        self.front_left_tire = FrontLeftTire(spec)
        self.front_right_tire = FrontRightTire(spec)
        self.rear_left_tire = RearLeftTire(spec)
        self.rear_right_tire = RearRightTire(spec)
        self.front_axle = FrontAxle(spec)
        self.rear_axle = RearAxle(spec)

        self.draw_area_width = draw_area_width

        self.trajectory = trajectory
        self.controller = controller

    def update(self, time_s):
        if not self.controller: return

        updated_state = self.state.update(0.0, 0.17, time_s, self.spec.wheel_base_m)
        self.state = updated_state

        updated_history = self.history.update(updated_state.get_x_m(), updated_state.get_y_m())
        self.history = updated_history
    
    def draw(self, axes, elems):
        x_y_yaw_array = self.state.x_y_yaw()
        x_m = self.state.get_x_m()
        y_m = self.state.get_y_m()

        elems += self.body.draw(axes, x_y_yaw_array)
        elems += self.chassis.draw(axes, x_y_yaw_array)
        elems += self.front_left_tire.draw(axes, x_y_yaw_array, 0.17)
        elems += self.front_right_tire.draw(axes, x_y_yaw_array, 0.17)
        elems += self.rear_left_tire.draw(axes, x_y_yaw_array, 0.0)
        elems += self.rear_right_tire.draw(axes, x_y_yaw_array, 0.0)
        elems += self.front_axle.draw(axes, x_y_yaw_array)
        elems += self.rear_axle.draw(axes, x_y_yaw_array)
        elems += self.history.draw(axes, self.spec.color)

        axes.set_xlim(x_m - self.draw_area_width, x_m + self.draw_area_width)
        axes.set_ylim(y_m - self.draw_area_width, y_m + self.draw_area_width)


def main():
    vis = GlobalXYVisualizer(time_span_s=20)

    spec = VehicleSpecification()
    state = State(15.0, 0.0, 0.0, 2.0)
    history = StateHistory([state.get_x_m()], [state.get_y_m()])
    vehicle = FourWheelsVehicle(state, history, spec)
    
    vis.add_object(vehicle)
    vis.draw()


if __name__ == "__main__":
    main()
