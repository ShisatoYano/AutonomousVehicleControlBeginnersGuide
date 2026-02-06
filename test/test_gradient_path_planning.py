"""
Test of gradient descent path planning simulation

Author: Panav Arpit Raaj
"""

from pathlib import Path
import sys

sys.path.append(str(Path(__file__).absolute().parent)
                + "/../src/simulations/path_planning/gradient_path_planning")
import gradient_path_planning


def test_simulation():
    gradient_path_planning.show_plot = False
    gradient_path_planning.main()
