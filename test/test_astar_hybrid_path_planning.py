"""
Test of A* path planning and navigation simulation

Author: Shreyansh Shethia
"""

from pathlib import Path
import sys
import pytest

sys.path.append(str(Path(__file__).absolute().parent) + "/../src/simulations/path_planning/astar_hybrid_path_planning")
import astar_hybrid_path_planning


def test_simulation():
    astar_hybrid_path_planning.show_plot = False
    astar_hybrid_path_planning.main()