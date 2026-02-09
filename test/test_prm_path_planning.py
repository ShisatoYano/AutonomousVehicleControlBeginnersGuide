"""
Test of PRM path planning and navigation simulation

Author: Erwin Lejeune
"""

from pathlib import Path
import sys
import pytest

sys.path.append(str(Path(__file__).absolute().parent) + "/../src/simulations/path_planning/prm_path_planning")
import prm_path_planning


def test_simulation():
    prm_path_planning.show_plot = False

    prm_path_planning.main()
