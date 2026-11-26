"""
Test of D* path planning simulation

Verifies both the initial search and the incremental replanning
after dynamic obstacles are injected.
"""

from pathlib import Path
import sys
import pytest

sys.path.append(str(Path(__file__).absolute().parent) + "/../src/simulations/path_planning/dstar_path_planning")
import dstar_path_planning


def test_simulation(tmp_path):
    dstar_path_planning.show_plot = False
    dstar_path_planning.output_dir = str(tmp_path)

    dstar_path_planning.main()

    assert (tmp_path / "map.json").exists()
    assert (tmp_path / "path.json").exists()
