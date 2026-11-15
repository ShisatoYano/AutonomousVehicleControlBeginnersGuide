"""Compatibility for the vehicle chassis."""

import numpy as np

from VehiclePart import VehiclePart


def Chassis(spec):
    """Create the vehicle chassis as a VehiclePart."""

    outline = np.array([[spec.f_len_m, -spec.r_len_m], [0.0, 0.0]])
    return VehiclePart(spec, outline, 0.0, 0.0)
