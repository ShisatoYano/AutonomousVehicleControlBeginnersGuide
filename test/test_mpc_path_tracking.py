"""
Test of path tracking simulation by MPC (Model Predictive Control) algorithm
"""

from pathlib import Path
import sys
import pytest

sys.path.append(
    str(Path(__file__).absolute().parent)
    + "/../src/simulations/path_tracking/mpc_path_tracking"
)

import mpc_path_tracking


def test_simulation():
    mpc_path_tracking.show_plot = False
    mpc_path_tracking.main()
