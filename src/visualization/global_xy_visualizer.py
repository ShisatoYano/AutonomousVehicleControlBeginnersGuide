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
    
    def __init__(self, x_min=0, x_max=30, y_min=-15, y_max=15, 
                 time_span_s=10, time_interval_s=0.1, save_gif_name=None):
        self.objects = [] # register objects here
        self.x_min = x_min
        self.x_max = x_max
        self.y_min = y_min
        self.y_max = y_max
        self.time_span_s = time_span_s
        self.time_interval_s = time_interval_s
        self.save_gif_name = save_gif_name
        self.show_plot = True
    
    def add_object(self, obj):
        """
        Method to register object
        """

        self.objects.append(obj)
    
    def not_show_plot(self):
        self.show_plot = False

    def update(self, i, elems, axes):
        while elems: elems.pop().remove()
        time_str = "Time = {0:.2f}[s]".format(self.time_interval_s * i)
        axes.set_title(time_str, fontsize=15)
        for obj in self.objects:
            obj.draw(axes, elems)
            if hasattr(obj, "update"): obj.update(self.time_interval_s)

        if self.time_interval_s * i >= self.time_span_s:
            axes.set_xlim(self.x_min, self.x_max)
            axes.set_ylim(self.y_min, self.y_max)

    def draw(self):
        # setting
        figure = plt.figure(figsize=(8, 8))
        axes = figure.add_subplot(111)
        axes.set_aspect("equal")
        axes.set_xlabel('X[m]', fontsize=20)
        axes.set_ylabel('Y[m]', fontsize=20)

        elems = []

        if self.show_plot:
            self.anime = anm.FuncAnimation(figure, self.update, fargs=(elems, axes),
                                           frames=int(self.time_span_s / self.time_interval_s)+1, 
                                           interval=int(self.time_interval_s * 1000), 
                                           repeat=False)
            
            if self.save_gif_name: self.anime.save(self.save_gif_name, writer="pillow")
            else: plt.show()
        else:
            for i in range(1000): self.update(i, elems, axes)


def main():
    vis = GlobalXYVisualizer()
    vis.draw()


if __name__ == "__main__":
    main()
