"""
four_wheels_vehicle.py

Author: Shisato Yano
"""

from body import Body
from chassis import Chassis
from front_left_tire import FrontLeftTire
from front_right_tire import FrontRightTire
from rear_left_tire import RearLeftTire
from rear_right_tire import RearRightTire
from front_axle import FrontAxle
from rear_axle import RearAxle


class FourWheelsVehicle:
    """
    Four Wheels Vehicle model class
    """

    def __init__(self, state, spec, controller=None, sensor=None):
        """
        Constructor
        state: Vehicle's state object
        spec: Vehicle's specification object
        draw_area_width: Drawing area's width around Vehicle
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

        self.controller = controller
        self.sensor = sensor

    def update(self, time_s):
        if self.sensor: self.sensor.update(self.state.x_y_yaw())

        if self.controller:
            self.controller.update(self.state)
            target_accel = self.controller.get_target_accel_mps2()
            target_yaw_rate = self.controller.get_target_yaw_rate_rps()
        else:
            target_accel = 0.0
            target_yaw_rate = 0.0

        self.state.update(target_accel, target_yaw_rate, time_s)
    
    def draw(self, axes, elems):
        self.state.draw(axes, elems)
        x_y_yaw_array = self.state.x_y_yaw()
        x_m = self.state.get_x_m()
        y_m = self.state.get_y_m()

        if self.sensor: self.sensor.draw(axes, x_y_yaw_array, elems)

        if self.controller:
            self.controller.draw(axes, elems)
            steer_rad = self.controller.get_target_steer_rad()
        else:
            steer_rad = 0.0
        
        self.body.draw(axes, x_y_yaw_array, elems)
        self.chassis.draw(axes, x_y_yaw_array, elems)
        self.front_left_tire.draw(axes, x_y_yaw_array, steer_rad, elems)
        self.front_right_tire.draw(axes, x_y_yaw_array, steer_rad, elems)
        self.rear_left_tire.draw(axes, x_y_yaw_array, elems)
        self.rear_right_tire.draw(axes, x_y_yaw_array, elems)
        self.front_axle.draw(axes, x_y_yaw_array, elems)
        self.rear_axle.draw(axes, x_y_yaw_array, elems)

        axes.set_xlim(x_m - self.spec.area_size, x_m + self.spec.area_size)
        axes.set_ylim(y_m - self.spec.area_size, y_m + self.spec.area_size)
