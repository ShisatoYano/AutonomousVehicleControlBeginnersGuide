"""
Test of Path tracking simulation by Stanley steering control algorithm

Author: Shisato Yano
"""

from pathlib import Path
import sys
import pytest

sys.path.append(str(Path(__file__).absolute().parent) + "/../src/simulations/path_tracking/stanley_path_tracking")
import stanley_path_tracking


def test_simulation():
    stanley_path_tracking.show_plot = False

    stanley_path_tracking.main()
