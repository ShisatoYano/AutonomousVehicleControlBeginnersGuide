"""
Test of Automated Sensor Calibration by UKF

Author: Shisato Yano
"""

from pathlib import Path
import sys
import pytest

sys.path.append(str(Path(__file__).absolute().parent) + "/../src/simulations/perception/sensor_auto_calibration")
import sensor_auto_calibration


def test_simulation():
    sensor_auto_calibration.show_plot = False

    sensor_auto_calibration.main()
