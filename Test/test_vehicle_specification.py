"""
Unit test of VehicleSpecification

Author: Shisato Yano
"""

import pytest
import sys
from pathlib import Path

sys.path.append(str(Path(__file__).absolute().parent) + "/../src/vehicle")
from vehicle_specification import VehicleSpecification


# test instance
spec = VehicleSpecification()


def test_create_instance():
    assert spec.f_len_m == 2.0
    assert spec.f_edge_m == 2.5

    assert spec.r_len_m == 0.0
    assert spec.r_edge_m == 0.5

    assert spec.tread_m == 0.75
    assert spec.width_m == 0.75
    assert spec.wheel_base_m == 2.0

    assert spec.tire_r_m == 0.3
    assert spec.tire_w_m == 0.12
    assert spec.axle_half_m == 0.5
    
    assert spec.color == 'k'
    assert spec.line_w == 1.0
    assert spec.line_type == '-'
    assert spec.area_size == 10.0