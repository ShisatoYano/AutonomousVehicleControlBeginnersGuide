"""Compatibility for the rear-right tire."""

from rear_left_tire import RearLeftTire


def RearRightTire(spec):
    """Create the rear right tire as a non-steerable VehiclePart."""

    tire = RearLeftTire(spec)
    tire.offset_y_m = -spec.axle_half_m
    return tire
