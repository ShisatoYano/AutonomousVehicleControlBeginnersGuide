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

    def __init__(self, state, spec, controller=None, sensors=None,
                 detector=None):
        """
        Constructor
        state: Vehicle's state object
        spec: Vehicle's specification object
        controller: Controller object
        sensors: Sencors object
        detector: Detector object
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

        self.detector = detector

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
    
    def _update_detection_data(self):
        """
        Private function to update each detected object data
        """
        
        if self.detector: self.detector.update(self.sensors.get_point_cloud_from_lidar())
    
    def _draw_detection_data(self, axes, elems, state):
        """
        Private function to draw each detected object data
        axes: Axes object of figure
        elems: List of plot object
        state: Vehicle's state object
        """
        
        if self.detector:
            self.detector.draw(axes, 
                               elems,
                               self.sensors.get_lidar_global_x_m(),
                               self.sensors.get_lidar_global_y_m(),
                               state.get_yaw_rad())

    def _update_control_data(self):
        """
        Private function to update controller's data
        Return 1: Acceleration input[m/s2]
        Return 2: Yaw rate input[rad/s]
        """
        
        if self.controller:
            self.controller.update(self.state)
            return self.controller.get_target_accel_mps2(), \
                self.controller.get_target_yaw_rate_rps()
        else:
            return 0.0, 0.0
    
    def _draw_control_data(self, axes, elems):
        """
        Private function to draw controller's data
        axes: Axes object of figure
        elems: List of plot object
        Return: Steering angle value[rad]
        """
        
        if self.controller:
            self.controller.draw(axes, elems)
            return self.controller.get_target_steer_rad()
        else:
            return 0.0

    def update(self, time_s):
        """
        Function to update each member objects
        time_s: Simulation interval time[sec]
        """

        self._update_sensors_data(self.state)

        self._update_detection_data()

        target_accel, target_yaw_rate = self._update_control_data()

        self.state.update(target_accel, target_yaw_rate, time_s)
    
    def draw(self, axes, elems):
        """
        Function to draw each member object's data
        axes: Axes object of figure
        elems: List of plot object
        """

        self._draw_sensors_data(axes, elems, self.state)

        self._draw_detection_data(axes, elems, self.state)

        steer_rad = self._draw_control_data(axes, elems)

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
