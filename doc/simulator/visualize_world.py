"""
visualize_world.py

Author: Shisato Yano
"""

# import path setting
import numpy as np
import sys
from pathlib import Path

abs_dir_path = str(Path(__file__).absolute().parent)
relative_path = "/../../src/components/"

sys.path.append(abs_dir_path + relative_path + "visualization")


# import component modules
from global_xy_visualizer import GlobalXYVisualizer
from min_max import MinMax
from time_parameters import TimeParameters


# flag to show plot figure
# when executed as unit test, this flag is set as false
show_plot = True


def main():
    """
    Main process function
    """
    
    # set simulation parameters
    x_lim, y_lim = MinMax(-30, 30), MinMax(-30, 30)
    vis = GlobalXYVisualizer(x_lim, y_lim, TimeParameters(span_sec=20))

    # add objects here

    # plot figure is not shown when executed as unit test
    if not show_plot: vis.not_show_plot()

    # show plot figure
    vis.draw()


# execute main process
if __name__ == "__main__":
    main()
