"""
Test of A* path planning and navigation simulation

Author: Shisato Yano
"""

from pathlib import Path
import sys
import pytest

sys.path.append(str(Path(__file__).absolute().parent) + "/../src/simulations/path_planning/astar_path_planning")
import astar_path_planning


def test_simulation():
    astar_path_planning.show_plot = False

    astar_path_planning.main()
