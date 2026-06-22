"""
Test of GA path planning and navigation simulation

Author: Banaan Kiamanesh
GitHub: https://github.com/BanaanKiamanesh
"""

from pathlib import Path
import sys

sys.path.append(
    str(Path(__file__).absolute().parent)
    + "/../src/simulations/path_planning/ga_path_planning"
)
import ga_path_planning  # noqa: E402


def test_simulation():
    ga_path_planning.show_plot = False

    ga_path_planning.main()
