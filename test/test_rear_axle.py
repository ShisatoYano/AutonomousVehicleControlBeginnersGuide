"""
Unit test of RearAxle

Author: Shisato Yano
"""

import sys
from pathlib import Path

sys.path.append(str(Path(__file__).absolute().parent) + "/../src/components/vehicle")
from rear_axle import RearAxle
from rear_left_tire import RearLeftTire
from rear_right_tire import RearRightTire
from vehicle_specification import VehicleSpecification


# non-zero r_len_m so the bug (fixed on the wrong side of the origin) is visible
spec = VehicleSpecification(r_len_m=1.0)
rear_axle = RearAxle(spec)
rear_left_tire = RearLeftTire(spec)
rear_right_tire = RearRightTire(spec)


def test_rear_axle_shares_x_offset_with_rear_tires():
    assert rear_axle.offset_x_m == rear_left_tire.offset_x_m
    assert rear_axle.offset_x_m == rear_right_tire.offset_x_m
    assert rear_axle.offset_x_m == -spec.r_len_m
