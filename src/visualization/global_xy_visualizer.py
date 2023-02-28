"""
global_xy_visualizer.py

Author: Shisato Yano
"""

import matplotlib.pyplot as plt
import matplotlib.animation as anm


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

    def one_step(self, i, elems, axes):
        while elems: elems.pop().remove()
        elems.append(axes.text(-4.4, 4.5, "t = "+str(i)+"[s]", fontsize=15))

    def draw(self):
        # setting
        figure = plt.figure(figsize=(8, 8))
        axes = figure.add_subplot(111)
        axes.set_aspect("equal")
        axes.set_xlim(self.min_lim, self.max_lim)
        axes.set_ylim(self.min_lim, self.max_lim)
        axes.set_xlabel('X[m]', fontsize=20)
        axes.set_ylabel('Y[m]', fontsize=20)

        elems = []

        if self.show_plot:
            self.anime = anm.FuncAnimation(figure, self.one_step, fargs=(elems, axes),
                                           frames=10, interval=1000, repeat=False)
            plt.show()
        else:
            for i in range(1000): self.one_step(i, elems, axes)

        # draw registered objects
        # for obj in self.objects: obj.draw(axes)

        # if self.show_plot: plt.show()


def main():
    vis = GlobalXYVisualizer()
    vis.draw()


if __name__ == "__main__":
    main()
