"""
Unit test of ExtendedKalmanFilterLocalizer

Author: Shisato Yano
"""

import matplotlib as mpl
mpl.use("Agg")
import matplotlib.pyplot as plt
import pytest
import sys
from pathlib import Path

sys.path.append(str(Path(__file__).absolute().parent) + "/../src/components/localization/kalman_filter")
from extended_kalman_filter_localizer import ExtendedKalmanFilterLocalizer
