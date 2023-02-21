"""
global_xy_visualizer.py

Author: Shisato Yano
"""

import matplotlib.pyplot as plt


class GlobalXYVisualizer:
    """
    Visualization class for global 2D X-Y plot
    """
    
    def __init__(self, a_min=-5, a_max=5):
        self.o_objects = [] # register objects here
        self.o_min = a_min
        self.o_max = a_max
        self.o_show_plot = True
    
    def add_object(self, a_object):
        """
        Method to register object
        """

        self.o_objects.append(a_object)
    
    def not_show_plot(self):
        self.o_show_plot = False

    def draw(self):
        # setting
        figure = plt.figure(figsize=(8, 8))
        axes = figure.add_subplot(111)
        axes.set_aspect("equal")
        axes.set_xlim(self.o_min, self.o_max)
        axes.set_ylim(self.o_min, self.o_max)
        axes.set_xlabel('X[m]', fontsize=20)
        axes.set_ylabel('Y[m]', fontsize=20)

        # draw registered objects
        for obj in self.o_objects: obj.draw()

        if self.o_show_plot: plt.show()


def main():
    vis = GlobalXYVisualizer()
    vis.draw()


if __name__ == "__main__":
    main()
