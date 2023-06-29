"""
Test of Path tracking simulation by Pure pursuit algorithm

Author: Shisato Yano
"""

from pathlib import Path
import sys
import pytest

sys.path.append(str(Path(__file__).absolute().parent) + "/../src/simulations/path_tracking/pure_pursuit_path_tracking")
import pure_pursuit_path_tracking


def test_simulation():
    pure_pursuit_path_tracking.show_plot = False

    pure_pursuit_path_tracking.main()
