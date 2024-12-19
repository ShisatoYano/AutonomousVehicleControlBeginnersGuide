"""
Test of Path tracking simulation by Rear wheel feedback algorithm

Author: Shisato Yano
"""

from pathlib import Path
import sys
import pytest

sys.path.append(str(Path(__file__).absolute().parent) + "/../src/simulations/path_tracking/rear_wheel_feedback_tracking")
import rear_wheel_feedback_tracking


def test_simulation():
    rear_wheel_feedback_tracking.show_plot = False

    rear_wheel_feedback_tracking.main()
