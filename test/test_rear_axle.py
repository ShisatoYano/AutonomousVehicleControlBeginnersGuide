"""
Unit test of the rear axle and rear tires of FourWheelsVehicle

Author: Dipak Chaudhari
"""

import numpy as np
import sys
from pathlib import Path

sys.path.append(str(Path(__file__).absolute().parent) + "/../src/components/vehicle")
sys.path.append(str(Path(__file__).absolute().parent) + "/../src/components/state")
from four_wheels_vehicle import FourWheelsVehicle
from state import State
from vehicle_specification import VehicleSpecification


class MockAxes:
    """
    Axes stand-in that records the x data of each plotted line
    """

    def __init__(self):
        self.plotted_x = []

    def plot(self, x_data, y_data, **kwargs):
        self.plotted_x.append(np.array(x_data))
        return (None,)


spec = VehicleSpecification(r_len_m=1.5)
vehicle = FourWheelsVehicle(State(), spec)


def test_rear_axle_shares_rear_tires_offset():
    assert vehicle.rear_axle.offset_x_m == -spec.r_len_m
    assert vehicle.rear_axle.offset_x_m == vehicle.rear_left_tire.offset_x_m
    assert vehicle.rear_axle.offset_x_m == vehicle.rear_right_tire.offset_x_m


def test_rear_axle_is_drawn_behind_origin():
    axes = MockAxes()
    vehicle.rear_axle.draw(
        axes, np.array([[0.0], [0.0], [0.0]]), 0.0, []
    )
    assert np.allclose(axes.plotted_x[0], -spec.r_len_m)
