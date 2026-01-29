"""
Test of path tracking simulation by Adaptive Pure Pursuit algorithm
"""

from pathlib import Path
import sys
import pytest

sys.path.append(
    str(Path(__file__).absolute().parent)
    + "/../src/simulations/path_tracking/adaptive_pure_pursuit_path_tracking"
)
import adaptive_pure_pursuit_path_tracking


def test_simulation():
    adaptive_pure_pursuit_path_tracking.show_plot = False

    adaptive_pure_pursuit_path_tracking.main()
