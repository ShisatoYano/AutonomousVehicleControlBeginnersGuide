"""
Test of ACO path planning and navigation simulation

Author: Banaan Kiamanesh
GitHub: https://github.com/BanaanKiamanesh
"""

from pathlib import Path
import sys

sys.path.append(
    str(Path(__file__).absolute().parent)
    + "/../src/simulations/path_planning/aco_path_planning"
)
import aco_path_planning  # noqa: E402


def test_simulation(tmp_path):
    aco_path_planning.show_plot = False
    aco_path_planning.output_dir = str(tmp_path)

    aco_path_planning.main()

    assert (tmp_path / "map.json").exists()
    assert (tmp_path / "path.json").exists()
