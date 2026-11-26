"""
Test of Informed RRT* path planning and navigation simulation

Author: Rajat Arora
"""

from pathlib import Path
import sys
import pytest

sys.path.append(str(Path(__file__).absolute().parent) + "/../src/simulations/path_planning/informed_rrt_star_path_planning")
import informed_rrt_star_path_planning


def test_simulation(tmp_path):
    informed_rrt_star_path_planning.show_plot = False
    informed_rrt_star_path_planning.output_dir = str(tmp_path)

    informed_rrt_star_path_planning.main()

    assert (tmp_path / "map.json").exists()
    assert (tmp_path / "path.json").exists()
