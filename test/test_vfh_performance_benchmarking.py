"""
Test of VFH performance benchmarking simulation

Author: Khushi
"""

from pathlib import Path
import sys

sys.path.append(str(Path(__file__).absolute().parent) + "/../src/simulations/mapping/vfh_performance_benchmarking")
import vfh_performance_benchmarking


def test_simulation():
    vfh_performance_benchmarking.show_plot = False

    vfh_performance_benchmarking.main()
