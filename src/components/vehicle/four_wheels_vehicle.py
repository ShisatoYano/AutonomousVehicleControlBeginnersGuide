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

    def __init__(self, state, spec, controller=None, sensors=None):
        """
        Constructor
        state: Vehicle's state object
        spec: Vehicle's specification object
        controller: Controller object
        sensors: Sencors object
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

        self.sensors = sensors
        self._install_sensors(self.state)

    def _install_sensors(self, state):
        """
        Private function to calculate each sensor's installation position on vehicle
        state: Vehicle's state object
        """

        if self.sensors: self.sensors.install(state)
    
    def _update_sensors_data(self, state):
        """
        Private function to update each sensor's data
        state: Vehicle's state object
        """

        if self.sensors: self.sensors.update_data(state)
    
    def _draw_sensors_data(self, axes, elems, state):
        """
        Private function to draw each sensor's data
        axes: Axes object of figure
        elems: List of plot object
        state: Vehicle's state object
        """

        if self.sensors: self.sensors.draw_data(axes, elems, state)

    def update(self, time_s):
        """
        Function to update each member objects
        time_s: Simulation interval time[sec]
        """

        self._update_sensors_data(self.state)

        if self.controller:
            self.controller.update(self.state)
            target_accel = self.controller.get_target_accel_mps2()
            target_yaw_rate = self.controller.get_target_yaw_rate_rps()
        else:
            target_accel = 0.0
            target_yaw_rate = 0.0

        self.state.update(target_accel, target_yaw_rate, time_s)
    
    def draw(self, axes, elems):
        """
        Function to draw each member object's data
        axes: Axes object of figure
        elems: List of plot object
        """

        self._draw_sensors_data(axes, elems, self.state)

        if self.controller:
            self.controller.draw(axes, elems)
            steer_rad = self.controller.get_target_steer_rad()
        else:
            steer_rad = 0.0

        self.state.draw(axes, elems)
        x_y_yaw_array = self.state.x_y_yaw()
        x_m = self.state.get_x_m()
        y_m = self.state.get_y_m()
        
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
