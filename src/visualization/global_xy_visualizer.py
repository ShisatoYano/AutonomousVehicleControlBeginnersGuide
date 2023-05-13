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
    
    def __init__(self, x_lim, y_lim, time_params, gif_name=None):
        self.objects = [] # register objects here
        self.x_lim = x_lim
        self.y_lim = y_lim
        self.time_params = time_params
        self.gif_name = gif_name
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

        time_str = "Time = {0:.2f}[s]".format(self.time_params.current_sec(i))
        axes.set_title(time_str, fontsize=15)
        
        for obj in self.objects:
            obj.draw(axes, elems)
            if hasattr(obj, "update"): obj.update(self.time_params.get_interval_sec())

        if self.time_params.simulation_finished(i):
            axes.set_xlim(self.x_lim.min_value(), self.x_lim.max_value())
            axes.set_ylim(self.y_lim.min_value(), self.y_lim.max_value())

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
                                           frames=self.time_params.get_frame_num(), 
                                           interval=self.time_params.get_interval_msec(), 
                                           repeat=False)
            
            if self.gif_name: self.anime.save(self.gif_name, writer="pillow")
            else: plt.show()
        else:
            for i in range(1000): self.update(i, elems, axes)


def main():
    vis = GlobalXYVisualizer()
    vis.draw()


if __name__ == "__main__":
    main()
