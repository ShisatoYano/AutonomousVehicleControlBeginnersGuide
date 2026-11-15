"""Compatibility for the front axle."""

import numpy as np

from VehiclePart import VehiclePart


def FrontAxle(spec):
    """Create the front axle as a VehiclePart."""

    outline = np.array([[0.0, 0.0], [spec.axle_half_m, -spec.axle_half_m]])
    return VehiclePart(spec, outline, spec.f_len_m, 0.0)
