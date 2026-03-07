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


def test_simulation():
    dstar_path_planning.show_plot = False

    dstar_path_planning.main()
