"""
Test of Informed RRT* path planning and navigation simulation

Author: Rajat Arora
"""

from pathlib import Path
import sys
import pytest

sys.path.append(str(Path(__file__).absolute().parent) + "/../src/simulations/path_planning/informed_rrt_star_path_planning")
import informed_rrt_star_path_planning


def test_simulation():
    informed_rrt_star_path_planning.show_plot = False

    informed_rrt_star_path_planning.main()
