"""
Test of PSO path planning and navigation simulation

Author: Erwin Lejeune
"""

from pathlib import Path
import sys
import pytest

sys.path.append(str(Path(__file__).absolute().parent) + "/../src/simulations/path_planning/pso_path_planning")
import pso_path_planning


def test_simulation():
    pso_path_planning.show_plot = False

    pso_path_planning.main()
