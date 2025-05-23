## 3.1 Obstacle
In this section, Obstacle class and Landmark class are implemented. These class are used for locating the obstacles object in the world. The located obstacles can be observed by an sensor model.   

### 3.1.1 Rectangle obstacle
The obstacle class is implemented as follow. [State class implemented in Chapter 2](/doc/2_vehicle_model/2_vehicle_model.md) is given to the constructor for computing the position and pose of the obstacle. And then, [XYArray class](/doc/2_vehicle_model/2_vehicle_model.md) is also imported in the class for representing the shape as rectangle.  

[obstacle.py](/src/components/obstacle/obstacle.py)  
```python
"""
obstacle.py

Author: Shisato Yano
"""

import numpy as np
import sys
from pathlib import Path

sys.path.append(str(Path(__file__).absolute().parent) + "/../array")
from xy_array import XYArray


class Obstacle:
    """
    Obstacle's data and logic class
    """

    def __init__(self, state, accel_mps2=0.0, yaw_rate_rps=0.0,
                 length_m=2.0, width_m=2.0):
        """
        Constructor
        state: Obstacle's state object
        accel_mps2: acceleration input to move[m/s2]
        yaw_rate_rps: yaw rate input to move[rad/s]
        length_m: Obstacle's longitudinal size[m]
        width_m: Obstacle's half of lateral size[m]
        """

        self.state = state
        self.accel_mps2 = accel_mps2
        self.yaw_rate_rps = yaw_rate_rps

        contour = np.array([[length_m, -length_m, -length_m, length_m, length_m],
                            [width_m, width_m, -width_m, -width_m, width_m]])
        self.array = XYArray(contour)
```

The given arguments, accel_mps2 and yaw_rate_rps are used for making the obstacle moved as a dynamic obstacle. If these values are default 0.0, the obstacle is represented as a static obstacle. Additionally, length_m and width_m are used for setting the size of the obstacle.  

The member methods, update, draw and vertex_xy are implemented. The method, update is used to compute the state including position(x, y) and yaw angle of the obstacle.  
```python
    def update(self, time_s):
        """
        Function to update obstacle's state
        time_s: Time interval value[sec]
        """
        
        self.state.update(self.accel_mps2, self.yaw_rate_rps, time_s)
```

The method, draw is used to visualize the obstacle in the world. The vertexes and lines of obstacle are transformed based on the state and visualized.  
```python
    def draw(self, axes, elems):
        """
        Function to draw obstacle
        axes: Axes object of figure
        elems: List of plot objects
        """
        
        x_m = self.state.get_x_m()
        y_m = self.state.get_y_m()
        yaw_rad = self.state.get_yaw_rad()

        transformed_array = self.array.homogeneous_transformation(x_m, y_m, yaw_rad)
        obstacle_plot, = axes.plot(transformed_array.get_x_data(), 
                                   transformed_array.get_y_data(), 
                                   lw=1.0, color='k', ls='-')
        elems.append(obstacle_plot)
```

The method, vertex_xy is used to get the vertexes coordinates (x, y) of the obstacle and to generate a point cloud by the sensor models.  
```python
    def vertex_xy(self):
        """
        Function to get obstacle's vertex point coordinates
        """
        
        x_m = self.state.get_x_m()
        y_m = self.state.get_y_m()
        yaw_rad = self.state.get_yaw_rad()

        transformed_array = self.array.homogeneous_transformation(x_m, y_m, yaw_rad)

        return transformed_array.get_x_data(), transformed_array.get_y_data()
```

### 3.1.2 Obstacles list
ObstacleList class is implemented to execute update and draw the data of multiple obstacles sequencially. An object of this list is given to a visualizer object and a sensor model object.  

[obstacle_list.py](/src/components/obstacle/obstacle_list.py)  
```python
"""
obstacle_list.py

Author: Shisato Yano
"""

class ObstacleList:
    """
    Obstacles list class
    """
    
    def __init__(self):
        """
        Constructor
        """
        
        self.list = []
    
    def add_obstacle(self, obstacle):
        """
        Function to add obstacle object into list
        obstacle: Obstacle class's object
        """
        
        self.list.append(obstacle)
    
    def update(self, times_s):
        """
        Function to update each obstacle's state in list
        time_s: Time interval value[sec]
        """
        
        for obst in self.list: obst.update(times_s)
    
    def draw(self, axes, elems):
        """
        Function to draw each obstacles in list
        axes: Axes object of figure
        elems: List of plot objects
        """
        
        for obst in self.list: obst.draw(axes, elems)
    
    def get_list(self):
        """
        Function to get obstacle's list
        """
        
        return self.list
```

This class has a list to store each obstacle objects as a member. Each obstacles can be stored by using a method, add_obstacle. The data of the obstacles can be updated and drawn sequencially by using update and draw methods. Finally, get_list method is implemented to get the list for a sensor model object.  

### 3.1.3 Visualization
A sample program for visualizing some obstacles in the simulation world is implemented as follow. A static obstacle and two dynamic obstacles can be drawn by giving an acceleration input and a yaw rate input.  

[visualize_obstacle.py](/doc/3_sensor_models/visualize_obstacle.py)  
```python
"""
visualize_obstacle.py

Author: Shisato Yano
"""

# import path setting
import numpy as np
import sys
from pathlib import Path

abs_dir_path = str(Path(__file__).absolute().parent)
relative_path = "/../../src/components/"

sys.path.append(abs_dir_path + relative_path + "visualization")
sys.path.append(abs_dir_path + relative_path + "state")
sys.path.append(abs_dir_path + relative_path + "vehicle")
sys.path.append(abs_dir_path + relative_path + "obstacle")


# import component modules
from global_xy_visualizer import GlobalXYVisualizer
from min_max import MinMax
from time_parameters import TimeParameters
from vehicle_specification import VehicleSpecification
from state import State
from four_wheels_vehicle import FourWheelsVehicle
from obstacle import Obstacle
from obstacle_list import ObstacleList


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

    # create obstacle instances
    obst_list = ObstacleList()
    obst1 = Obstacle(State(x_m=-5.0, y_m=15.0, speed_mps=1.0), yaw_rate_rps=np.deg2rad(10), width_m=1.0)
    obst_list.add_obstacle(obst1)
    obst2 = Obstacle(State(x_m=-15.0, y_m=-15.0), length_m=10.0, width_m=5.0)
    obst_list.add_obstacle(obst2)
    obst3 = Obstacle(State(x_m=20.0), yaw_rate_rps=np.deg2rad(15))
    obst_list.add_obstacle(obst3)
    vis.add_object(obst_list)

    # create vehicle instance
    spec = VehicleSpecification(area_size=30.0) # spec instance
    vehicle = FourWheelsVehicle(State(color=spec.color), spec) # set state, spec as arguments
    vis.add_object(vehicle)

    # plot figure is not shown when executed as unit test
    if not show_plot: vis.not_show_plot()

    # show plot figure
    vis.draw()


# execute main process
if __name__ == "__main__":
    main()
```

The following simulation can be visualized by executing the above program.  
![](/doc/3_sensor_models/visualize_obstacle.gif)  