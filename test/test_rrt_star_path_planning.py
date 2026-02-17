"""
Test of RRT* path planning and navigation simulation

Author: Auto-generated
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
    rrt_star_path_planning.show_plot = False

    rrt_star_path_planning.main()
