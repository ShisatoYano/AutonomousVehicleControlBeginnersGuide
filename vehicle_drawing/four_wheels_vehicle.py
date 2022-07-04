"""
Four wheels vehicle drawing program

Author: Shisato Yano
"""

import matplotlib.pyplot as plt

from body import Body
from tire import Tire
from chassis import Chassis
from axle import Axle

FRONT_LENGHT_M = 6.35
REAR_LENGTH_M = 0.0
TIRE_RADIUS_M = 1.27
TIRE_WIDTH_HALF_M = 0.64
AXLE_HALF_M = 2.54


class FourWheelsVehicle:
    def __init__(self, axes):
        self.body = Body(axes, FRONT_LENGHT_M, REAR_LENGTH_M)
        self.chassis = Chassis(axes, FRONT_LENGHT_M, REAR_LENGTH_M)
        self.front_axle = Axle(axes, FRONT_LENGHT_M, AXLE_HALF_M)
        self.rear_axle = Axle(axes, -REAR_LENGTH_M, AXLE_HALF_M)
        self.front_left_tire = Tire(axes, TIRE_RADIUS_M, TIRE_WIDTH_HALF_M, FRONT_LENGHT_M, AXLE_HALF_M)
        self.front_right_tire = Tire(axes, TIRE_RADIUS_M, TIRE_WIDTH_HALF_M, FRONT_LENGHT_M, -AXLE_HALF_M)
        self.rear_left_tire = Tire(axes, TIRE_RADIUS_M, TIRE_WIDTH_HALF_M, -REAR_LENGTH_M, AXLE_HALF_M)
        self.rear_right_tire = Tire(axes, TIRE_RADIUS_M, TIRE_WIDTH_HALF_M, -REAR_LENGTH_M, -AXLE_HALF_M)
    
    def draw(self, x_m, y_m, yaw_angle_deg, steer_angle_deg):
        self.body.draw(x_m, y_m, yaw_angle_deg)
        self.chassis.draw(x_m, y_m, yaw_angle_deg)
        self.front_axle.draw(x_m, y_m, yaw_angle_deg, 0.0)
        self.rear_axle.draw(x_m, y_m, yaw_angle_deg, 0.0)
        self.front_left_tire.draw(x_m, y_m, yaw_angle_deg, steer_angle_deg)
        self.front_right_tire.draw(x_m, y_m, yaw_angle_deg, steer_angle_deg)
        self.rear_left_tire.draw(x_m, y_m, yaw_angle_deg, 0.0)
        self.rear_right_tire.draw(x_m, y_m, yaw_angle_deg, 0.0)


if __name__ == "__main__":
    print(__file__ + " start!!")

    ax = plt.subplot(1, 1, 1)
    ax.set_xlabel("X[m]")
    ax.set_ylabel("Y[m]")
    ax.set_xlim([-10, 10])
    ax.set_ylim([-10, 10])
    ax.set_aspect("equal")
    ax.grid(True)

    fwv = FourWheelsVehicle(ax)

    angle_deg = -60
    while angle_deg <= 60:
        fwv.draw(0, 0, angle_deg, angle_deg)

        angle_deg += 30
        
        plt.pause(1)
