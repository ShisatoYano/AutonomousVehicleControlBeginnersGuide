"""Compatibility for the rear-left tire."""

from front_left_tire import FrontLeftTire


def RearLeftTire(spec):
    """Create the rear left tire as a non-steerable VehiclePart."""

    tire = FrontLeftTire(spec)
    tire.offset_x_m = -spec.r_len_m
    tire.steerable = False
    return tire
