# 1. World Visualization
In this chapter, I'm gonna explain about a program to define a world coordinate as an environment where a vehicle can move. The program can display a x-y world coordinates and manage each objects which exist on the world.  

## 1.1 Visualization class
All classes for world visualization is located at a directory, "src/components/visualization". Firstly, I'm gonna introduce a global x-y coordinate visualization class as follow.  
[global_xy_visualizer.py](/src/components/visualization/global_xy_visualizer.py)  
```python
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
    
    def __init__(self, x_lim, y_lim, time_params, show_zoom=True, gif_name=None):
        """
        Constructor
        x_lim: MinMax object of x axis
        y_lim: MinMax object of y axis
        time_params: TimeParameters object
        show_zoom: Zoom around vehicle when this flag is true
        gif_name: Gif file's name as string
        """
        
        self.objects = []

        self.x_lim = x_lim
        self.y_lim = y_lim
        
        self.time_params = time_params
        self.gif_name = gif_name
        self.show_plot = True
        self.show_zoom = show_zoom
```
This class need to import matplotlib.pyplot and animation to display data plots and an animation. In "__init__" method, an empty "objects" list is defined. Each objects which are located in the world are stored into this list.   

### 1.1.1 MinMax class
"x_lim" and "y_lim" are objects of MinMax class. This class defines limitation  of x/y axis. These objects are used to set a size of world visualization. 
[min_max.py](/src/components/visualization/min_max.py)  
```python
"""
min_max.py

Author: Shisato Yano
"""

class MinMax:
    """
    Limitation values class of axis
    """

    def __init__(self, min=0, max=10):
        """
        Constructor
        min: Minimum value of axis
        max: Maximum value of axis
        """

        self.min = min
        self.max = max
    
    def min_value(self):
        """
        Function to get minimum value
        """
        
        return self.min
    
    def max_value(self):
        """
        Function to get maximum value
        """
        
        return self.max
```

### 1.1.2 TimeParameters class
"time_params" is an object of TimeParameters class to manage the visualization time.  
[time_parameters.py](/src/components/visualization/time_parameters.py)
```python
"""
time_parameters.py

Author: Shisato Yano
"""

class TimeParameters:
    """
    Parameters class for visualization time setting
    """

    def __init__(self, span_sec=10, interval_sec=0.1):
        self.span_sec = span_sec
        self.interval_sec = interval_sec
        self.interval_msec = interval_sec * 1000
        self.frame_num = int(span_sec / interval_sec) + 1
    
    def get_interval_sec(self):
        """
        Function to get interval time parameter[sec]
        """
        
        return self.interval_sec
    
    def get_interval_msec(self):
        """
        Function to get interval time parameter[msec]
        """
        
        return self.interval_msec
    
    def get_frame_num(self):
        """
        Function to get number of simulation's frames
        """
        
        return self.frame_num
    
    def current_sec(self, index):
        """
        Function to get current time[sec] in simulation
        index: Index of simulation's frame
        """
        return self.interval_sec * index
    
    def simulation_finished(self, index):
        """
        Function to check simulation was finished
        index: Index of simulation's frame
        """
        
        return (self.interval_sec * index >= self.span_sec)
```

### 1.1.3 Other member variables
"gif_name" is a string object for saving the animation's gif file. A boolean, "show_plot" is used to switch displaying or not displaying the visualization's figure window when unit test is executed. While the test is running, the figure window should not be displayed to continue the test. "show_zoom" is deciding to limit the size of visualization area until around of the vehicle or the maximum size of world.  

### 1.1.4 Member methods
A member method, "add_object()" is defined to add each objects which is located in the world. An object can be added to the objects list by calling this method.  
```python
    def add_object(self, obj):
        """
        Function to register object which should be visualized
        Each elements of list need to have update and draw functions
        """

        self.objects.append(obj)
```

A member method, "not_show_plot()" is defined not to display the figure window of the visualization. This method can be used when the unit test is executed.  
```python
    def not_show_plot(self):
        """
        Function to set show plot flag as false
        Each data is visualized only when this flag is false
        """
        
        self.show_plot = False
```

This member method, "update" is used to update each objects's data and draw the animation in the list periodically. For this process, each objects in the list must have "update" method and "draw" method.
```python
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
        if not self.show_zoom or self.time_params.simulation_finished(i):
            axes.set_xlim(self.x_lim.min_value(), self.x_lim.max_value())
            axes.set_ylim(self.y_lim.min_value(), self.y_lim.max_value())
```

Finally, this "draw" method is defined to execute simulation including updating and visualization. If a specific name of gif file is set to "gif_name", the simulation's gif file will be created and saved instead of visualization.
```python
    def draw(self):
        """
        Function to define animation's main process
        """

        # clear existing plot and close existing figure
        plt.clf()
        plt.close()
        
        # setting figure and axes
        figure = plt.figure(figsize=(8, 8))
        axes = figure.add_subplot(111)
        axes.set_aspect("equal")
        axes.set_xlabel('X[m]', fontsize=20)
        axes.set_ylabel('Y[m]', fontsize=20)

        # create animation instance
        elems = []
        if self.show_plot:
            print("Simulation start!!")
            self.anime = anm.FuncAnimation(figure, self.update, fargs=(elems,   axes),
                                           frames=self.time_params.get_frame_num(), 
                                           interval=self.time_params.get_interval_msec(), 
                                           repeat=False)
            if self.gif_name:
                self.anime.save(self.gif_name, writer="pillow")
            else:
                plt.show()
            print("Simulation finished!!")
        else:
            # only when executed as unit test
            for i in range(1000): self.update(i, elems, axes)
```

## 1.2 Visualize empty world
I prepared for [a sample progoram to visualize an empty world](/doc/1_world_visualization/visualize_world.py).  
```python
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
```
You will be able to understand how to use the above world visualization class by reading the code. By executing this, you can see that an empty world simulation is visualized as follow.  
![](/doc/1_world_visualization/visualize_world.gif)  
