"""
Test of Frenet optimal trajectory planning simulation

Author: Yuvraj Gupta
"""

from pathlib import Path
import sys
import pytest

sys.path.append(str(Path(__file__).absolute().parent) + "/../src/simulations/path_planning/frenet_optimal_trajectory_planning")
import frenet_optimal_trajectory_planning


def test_simulation():
    frenet_optimal_trajectory_planning.show_plot = False

    frenet_optimal_trajectory_planning.main()
