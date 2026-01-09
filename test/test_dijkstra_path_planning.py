"""
Test of Dijkstra path planning and navigation simulation

Author: Ashish Varma
"""

from pathlib import Path
import sys
import pytest

sys.path.append(str(Path(__file__).absolute().parent) + "/../src/simulations/path_planning/dijkstra_path_planning")
import dijkstra_path_planning


def test_simulation():
    dijkstra_path_planning.show_plot = False

    dijkstra_path_planning.main()
