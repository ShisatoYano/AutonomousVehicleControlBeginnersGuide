"""
Test of A* path planning and navigation simulation

Author: Shreyansh Shethia
"""

from pathlib import Path
import sys
import pytest

sys.path.append(str(Path(__file__).absolute().parent) + "/../src/simulations/path_planning/astar_hybrid_path_planning")
import astar_hybrid_path_planning


def test_simulation(tmp_path):
    astar_hybrid_path_planning.show_plot = False
    astar_hybrid_path_planning.output_dir = str(tmp_path)
    astar_hybrid_path_planning.main()

    assert (tmp_path / "map.json").exists()
    assert (tmp_path / "path.json").exists()