"""Compatibility for the vehicle body."""

import numpy as np

from VehiclePart import VehiclePart


def Body(spec):
    """Create the vehicle body as a VehiclePart."""

    outline = np.array([
        [spec.f_edge_m, -spec.r_edge_m, -spec.r_edge_m, spec.f_edge_m, spec.f_edge_m],
        [spec.width_m, spec.width_m, -spec.width_m, -spec.width_m, spec.width_m],
    ])
    return VehiclePart(spec, outline, 0.0, 0.0)
