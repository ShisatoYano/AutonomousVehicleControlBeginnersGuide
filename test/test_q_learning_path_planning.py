"""
Test of Q-learning path planning and navigation simulation

Author: Banaan Kiamanesh
GitHub: https://github.com/BanaanKiamanesh
"""

from pathlib import Path
import sys

sys.path.append(
    str(Path(__file__).absolute().parent)
    + "/../src/simulations/path_planning/q_learning_path_planning"
)
import q_learning_path_planning  # noqa: E402


def test_simulation():
    q_learning_path_planning.show_plot = False

    q_learning_path_planning.main()
