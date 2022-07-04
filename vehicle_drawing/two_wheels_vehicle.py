"""
Two wheels vehicle drawing program

Author: Shisato Yano
"""

import matplotlib.pyplot as plt

from body import Body
from tire import Tire
from chassis import Chassis

FRONT_LENGHT_M = 6.35
REAR_LENGTH_M = 0.0
TIRE_RADIUS_M = 1.27
TIRE_WIDTH_HALF_M = 0.64


class TwoWheelsVehicle:
    def __init__(self, axes):
        self.body = Body(axes, FRONT_LENGHT_M, REAR_LENGTH_M)
        self.front_tire = Tire(axes, TIRE_RADIUS_M, TIRE_WIDTH_HALF_M, FRONT_LENGHT_M, 0.0)
        self.rear_tire = Tire(axes, TIRE_RADIUS_M, TIRE_WIDTH_HALF_M, -REAR_LENGTH_M, 0.0)
        self.chassis = Chassis(axes, FRONT_LENGHT_M, REAR_LENGTH_M)
    
    def draw(self, x_m, y_m, yaw_angle_deg, steer_angle_deg):
        self.body.draw(x_m, y_m, yaw_angle_deg)
        self.front_tire.draw(x_m, y_m, yaw_angle_deg, steer_angle_deg)
        self.rear_tire.draw(x_m, y_m, yaw_angle_deg, 0.0)
        self.chassis.draw(x_m, y_m, yaw_angle_deg)


if __name__ == "__main__":
    print(__file__ + " start!!")

    ax = plt.subplot(1, 1, 1)
    ax.set_xlabel("X[m]")
    ax.set_ylabel("Y[m]")
    ax.set_xlim([-10, 10])
    ax.set_ylim([-10, 10])
    ax.set_aspect("equal")
    ax.grid(True)

    twv = TwoWheelsVehicle(ax)

    angle_deg = -60
    while angle_deg <= 60:
        twv.draw(0, 0, angle_deg, angle_deg)

        angle_deg += 30
        
        plt.pause(1)
