"""
Unit test of RearAxle

Author: Dipak Chaudhari
"""

import numpy as np
import sys
from pathlib import Path

sys.path.append(str(Path(__file__).absolute().parent) + "/../src/components/vehicle")
from vehicle_specification import VehicleSpecification
from rear_axle import RearAxle
from rear_left_tire import RearLeftTire
from rear_right_tire import RearRightTire


class MockAxes:
    """
    Axes stand-in that records the x data of each plotted line
    """

    def __init__(self):
        self.plotted_x = []

    def plot(self, x_data, y_data, **kwargs):
        self.plotted_x.append(np.array(x_data))
        return (None,)


# vehicle whose rear axle is behind the origin
spec = VehicleSpecification(r_len_m=1.5)


def test_rear_axle_shares_rear_tires_offset():
    axle = RearAxle(spec)
    assert axle.offset_x_m == -spec.r_len_m
    assert axle.offset_x_m == RearLeftTire(spec).offset_x_m
    assert axle.offset_x_m == RearRightTire(spec).offset_x_m


def test_rear_axle_is_drawn_behind_origin():
    axes = MockAxes()
    RearAxle(spec).draw(axes, np.array([[0.0], [0.0], [0.0]]), [])
    assert np.allclose(axes.plotted_x[0], -spec.r_len_m)
