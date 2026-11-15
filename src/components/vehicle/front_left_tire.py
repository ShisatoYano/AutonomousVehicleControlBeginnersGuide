"""Compatibility for the front-left tire."""

import numpy as np

from VehiclePart import VehiclePart


def FrontLeftTire(spec):
    """Create the front left tire as a steerable VehiclePart."""

    outline = np.array([
        [spec.tire_r_m, -spec.tire_r_m, -spec.tire_r_m, spec.tire_r_m, spec.tire_r_m],
        [spec.tire_w_m, spec.tire_w_m, -spec.tire_w_m, -spec.tire_w_m, spec.tire_w_m],
    ])
    return VehiclePart(spec, outline, spec.f_len_m, spec.axle_half_m, True)
