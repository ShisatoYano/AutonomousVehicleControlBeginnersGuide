"""
Test of A* path planning and navigation simulation

Author: Yuvraj Gupta
"""
from pathlib import Path
import sys
import pytest

sys.path.append(
    str(Path(__file__).absolute().parent)
    + "/../src/simulations/path_planning/rrt_star_path_planning"
)

import rrt_star_path_planning


def test_simulation():
    # Disable visualization for CI / testing
    rrt_star_path_planning.show_plot = False

    # Run simulation (smoke test)
    rrt_star_path_planning.main()
