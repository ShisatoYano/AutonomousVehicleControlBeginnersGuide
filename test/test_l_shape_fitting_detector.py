"""
Unit test of PurePursuitController

Author: Shisato Yano
"""

import matplotlib as mpl
mpl.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pytest
import sys
from pathlib import Path

sys.path.append(str(Path(__file__).absolute().parent) + "/../src/components/detection/l_shape_fitting")
from l_shape_fitting_detector import LShapeFittingDetector


# test target instance
detector = LShapeFittingDetector()


def test_initialize():
    assert detector.MIN_RNG_TH_M == 3.0
    assert detector.RNG_TH_RATE == 0.1
    assert detector.CHANGE_ANGLE_RAD == np.deg2rad(1.0)
    assert len(detector.latest_rectangles_list) == 0
