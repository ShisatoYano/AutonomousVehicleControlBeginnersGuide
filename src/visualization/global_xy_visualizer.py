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
        """
        Constructor
        x_lim: MinMax object of x axis
        y_lim: MinMax object of y axis
        time_params: TimeParameters object
        gif_name: Name of saved gif file as string
        """
        
        self.objects = []

        self.x_lim = x_lim
        self.y_lim = y_lim
        
        self.time_params = time_params
        self.gif_name = gif_name
        self.show_plot = True
    
    def add_object(self, obj):
        """
        Function to register object which should be visualized
        Each elements of list need to have update and draw functions
        """

        self.objects.append(obj)
    
    def not_show_plot(self):
        """
        Function to set show plot flag as false
        Each data is visualized only when this flag is false
        """
        
        self.show_plot = False

    def update(self, i, elems, axes):
        """
        Function to update each objects and draw
        i: Index of animation frames
        elems: List of plot objects
        axes: Axes of figure
        """

        # remove each elements in list
        while elems: elems.pop().remove()

        # show current simulation time[sec] as title
        time_str = "Time = {0:.2f}[s]".format(self.time_params.current_sec(i))
        axes.set_title(time_str, fontsize=15)
        
        # draw and update each object's data
        for obj in self.objects:
            obj.draw(axes, elems)
            if hasattr(obj, "update"): obj.update(self.time_params.get_interval_sec())

        # show data between x-y min and max range
        if self.time_params.simulation_finished(i):
            axes.set_xlim(self.x_lim.min_value(), self.x_lim.max_value())
            axes.set_ylim(self.y_lim.min_value(), self.y_lim.max_value())

    def draw(self):
        """
        Function to define animation's main process
        """
        
        # setting figure and axes
        figure = plt.figure(figsize=(8, 8))
        axes = figure.add_subplot(111)
        axes.set_aspect("equal")
        axes.set_xlabel('X[m]', fontsize=20)
        axes.set_ylabel('Y[m]', fontsize=20)

        # create animation instance
        elems = []
        if self.show_plot:
            self.anime = anm.FuncAnimation(figure, self.update, fargs=(elems, axes),
                                           frames=self.time_params.get_frame_num(), 
                                           interval=self.time_params.get_interval_msec(), 
                                           repeat=False)
            # gif file is saved only when file's name as string is set
            # in this case, animation is not shown
            if self.gif_name: self.anime.save(self.gif_name, writer="pillow")
            else: plt.show()
        else:
            # only when executed as unit test
            for i in range(1000): self.update(i, elems, axes)
