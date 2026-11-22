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


spec = VehicleSpecification(r_len_m=1.5)
vehicle = FourWheelsVehicle(State(), spec)


def test_rear_axle_shares_rear_tires_offset():
    assert vehicle.rear_axle.offset_x_m == -spec.r_len_m
    assert vehicle.rear_axle.offset_x_m == vehicle.rear_left_tire.offset_x_m
    assert vehicle.rear_axle.offset_x_m == vehicle.rear_right_tire.offset_x_m


def test_rear_axle_is_drawn_behind_origin():
    outline_xy = vehicle.rear_axle.transformed_outline(
        np.array([[0.0], [0.0], [0.0]]), 0.0
    )
    assert np.allclose(outline_xy[0, :], -spec.r_len_m)
