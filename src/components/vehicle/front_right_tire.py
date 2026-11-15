"""Compatibility for the front-right tire."""

from front_left_tire import FrontLeftTire


def FrontRightTire(spec):
    """Create the front right tire as a steerable VehiclePart."""

    tire = FrontLeftTire(spec)
    tire.offset_y_m = -spec.axle_half_m
    return tire
