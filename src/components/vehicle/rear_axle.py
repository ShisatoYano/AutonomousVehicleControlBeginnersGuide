"""Compatibility for the rear axle."""

import numpy as np

from VehiclePart import VehiclePart


def RearAxle(spec):
    """Create the rear axle as a VehiclePart."""

    outline = np.array([[0.0, 0.0], [spec.axle_half_m, -spec.axle_half_m]])
    return VehiclePart(spec, outline, -spec.r_len_m, 0.0)
