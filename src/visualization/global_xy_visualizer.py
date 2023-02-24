"""
global_xy_visualizer.py

Author: Shisato Yano
"""

import matplotlib.pyplot as plt


class GlobalXYVisualizer:
    """
    Visualization class for global 2D X-Y plot
    """
    
    def __init__(self, min_lim=-5, max_lim=5):
        self.objects = [] # register objects here
        self.min_lim = min_lim
        self.max_lim = max_lim
        self.show_plot = True
    
    def add_object(self, obj):
        """
        Method to register object
        """

        self.objects.append(obj)
    
    def not_show_plot(self):
        self.show_plot = False

    def draw(self):
        # setting
        figure = plt.figure(figsize=(8, 8))
        axes = figure.add_subplot(111)
        axes.set_aspect("equal")
        axes.set_xlim(self.min_lim, self.max_lim)
        axes.set_ylim(self.min_lim, self.max_lim)
        axes.set_xlabel('X[m]', fontsize=20)
        axes.set_ylabel('Y[m]', fontsize=20)

        # draw registered objects
        for obj in self.objects: obj.draw(axes)

        if self.show_plot: plt.show()


def main():
    vis = GlobalXYVisualizer()
    vis.draw()


if __name__ == "__main__":
    main()
