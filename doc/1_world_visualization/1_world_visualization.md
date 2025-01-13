# 1. World Visualization
In this chapter, I'm gonna explain about a program to define a world coordinate as an environment where a vehicle can move. The program can display a x-y world coordinates and manage each objects which exist on the world.  

## 1.1 Visualization class
All classes for world visualization is located at a directory, "src/components/visualization". Firstly, I'm gonna introduce a global x-y coordinate visualization class as follow.  

src/components/visualization/global_xy_visualizer.py  
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

This class need to import matplotlib.pyplot and animation to display data plots and an animation. In "__init__" method, an empty "objects" list is defined. Each objects which are located in the world are stored into this list. "x_lim" and "y_lim" are limitation object of x/y axis. These objects are used to set a size of world visualization. "time_params" is an object to set the visualization time. A boolean, "show_plot" is used to switch displaying or not displaying the visualization's figure window when unit test is executed. While the test is running, the figure window should not be displayed to continue the test. "show_zoom" is deciding to limit the size of visualization area until around of the vehicle or the maximum size of world.  

A method, "add_object" is defined to add each objects which is located in the world. An object can be added to the objects list by calling this method.  
```python
    def add_object(self, obj):
        """
        Function to register object which should be visualized
        Each elements of list need to have update and draw functions
        """

        self.objects.append(obj)
```