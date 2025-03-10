## 3.3 Sensors
In this section, Sensors class is implemented for managing and processing data of each sensor models. This class can initialize, update and draw the data of each sensor models.  

### 3.3.1 Constructor
An object of [LiDAR class](/doc/3_sensor_models/3_2_lidar.md) is given to a constructor of this class as an argument. The argument is None as default and is stored as a member variable.  
```python
"""
sensors.py

Author: Shisato Yano
"""

class Sensors:
    """
    Each sensors logic class
    """
    
    def __init__(self, lidar=None):
        """
        Constructor
        lidar: lidar object
        """

        self.lidar = lidar
```

### 3.3.2 Installation
A function to install each sensors is implemented to compute the global position coordinate on the vehicle. It is computed only when the object of LiDAR class is given to the constructor.  
```python
    def install(self, state):
        """
        Function to calculate each sensor's installation position on vehicle
        state: vehicle's state object
        """

        if self.lidar: self.lidar.install(state)
```

### 3.3.3 Update and Draw
Two functions to update data and to draw data are implemented. These processes are executed only when the object of LiDAR class is installed.  
```python
    def update_data(self, state):
        """
        Function to update each sensor's data
        state: vehicle's state object
        """

        if self.lidar: self.lidar.update(state)
    
    def draw_data(self, axes, elems, state):
        """
        Function to draw each sensor's data
        axes: axes object of figure
        elems: list of plot object
        state: vehicle's state object
        """

        if self.lidar: self.lidar.draw(axes, elems, state)
```

### 3.3.4 Getters
The following three functions to get data of LiDAR are implemented. First one is to get an array of a point cloud. Second one is to get global x coordinate of the LiDAR. Third one is to get global y coordinate of the LiDAR.  
```python
    def get_point_cloud_from_lidar(self):
        """
        Function to get point cloud list from LiDAR
        If LiDAR was not installed, would return empty list
        """
        
        if self.lidar: return self.lidar.get_point_cloud()
        else: return []
    
    def get_lidar_global_x_m(self):
        """
        Function to get LiDAR's global position x[m]
        When LiDAR is not installed, return 0.0
        """
        
        if self.lidar: return self.lidar.get_global_x_m()
        else: return 0.0
    
    def get_lidar_global_y_m(self):
        """
        Function to get LiDAR's global position x[m]
        When LiDAR is not installed, return 0.0
        """

        if self.lidar: return self.lidar.get_global_y_m()
        else: return 0.0
```

### 3.3.5 Vehicle class
For installing the object of sensors class on the vehicle model, [Four wheels vehicle class](/src/components/vehicle/four_wheels_vehicle.py) need to be modified. Firstly, an argument, sensors is added to the constructor. This argument is None as default. When it is None, any processes are not executed.  
```python
class FourWheelsVehicle:
    """
    Four Wheels Vehicle model class
    """

    def __init__(self, state, spec, sensors=None, show_zoom=True):
        """
        Constructor
        state: Vehicle's state object
        spec: Vehicle's specification object
        sensors: Sencors object
        show_zoom: Flag for zoom around vehicle
        """
```

A private function to install sensors is implemented. This function is used to compute the global position coordinate of the sensors based on the current state of the vehicle. The function, install in Sensors class is called.  
```python
    def _install_sensors(self, state):
        """
        Private function to calculate each sensor's installation position on vehicle
        state: Vehicle's state object
        """

        if self.sensors: self.sensors.install(state)
```
The added argument, sensors is stored in a member variable and the above function, _install_sensors() is called in the constructor.  
```python
        self.sensors = sensors
        self._install_sensors(self.state)
```

For updating and drawing the data, the following two private functions are implemented.  
```python
    def _update_sensors_data(self, state):
        """
        Private function to update each sensor's data
        state: Vehicle's state object
        """

        if self.sensors: self.sensors.update_data(state)
    
    def _draw_sensors_data(self, axes, elems, state):
        """
        Private function to draw each sensor's data
        axes: Axes object of figure
        elems: List of plot object
        state: Vehicle's state object
        """

        if self.sensors: self.sensors.draw_data(axes, elems, state)
```
These functions are called in the update function and draw function of the vehicle class.  
```python
    def update(self, time_s):
        """
        Function to update each member objects
        time_s: Simulation interval time[sec]
        """

        self._update_sensors_data(self.state)

    def draw(self, axes, elems):
        """
        Function to draw each member object's data
        axes: Axes object of figure
        elems: List of plot object
        """

        self._draw_sensors_data(axes, elems, self.state)

        self.state.draw(axes, elems)
        x_y_yaw_array = self.state.x_y_yaw()
        x_m = self.state.get_x_m()
        y_m = self.state.get_y_m()
        
        self.body.draw(axes, x_y_yaw_array, elems)
        self.chassis.draw(axes, x_y_yaw_array, elems)
        self.front_left_tire.draw(axes, x_y_yaw_array, steer_rad, elems)
        self.front_right_tire.draw(axes, x_y_yaw_array, steer_rad, elems)
        self.rear_left_tire.draw(axes, x_y_yaw_array, elems)
        self.rear_right_tire.draw(axes, x_y_yaw_array, elems)
        self.front_axle.draw(axes, x_y_yaw_array, elems)
        self.rear_axle.draw(axes, x_y_yaw_array, elems)

        if self.show_zoom:
            axes.set_xlim(x_m - self.spec.area_size, x_m + self.spec.area_size)
            axes.set_ylim(y_m - self.spec.area_size, y_m + self.spec.area_size)
        else:
            axes.set_xlim(self.spec.x_lim.min_value(), self.spec.x_lim.max_value())
            axes.set_ylim(self.spec.y_lim.min_value(), self.spec.y_lim.max_value())
```

### 3.3.6 LiDAR Sensing Simulation
The object sesning with 2D LiDAR can be simulated by executing the following sample program. A single 2D LiDAR senses multiple obstacles and the point cloud data is generated.  
[lidar_obstacle_sensing.py](/src/simulations/perception/lidar_obstacle_sensing/lidar_obstacle_sensing.py)  
```python
"""
lidar_obstacle_sensing.py

Author: Shisato Yano
"""

# import path setting
import numpy as np
import sys
from pathlib import Path

abs_dir_path = str(Path(__file__).absolute().parent)
relative_path = "/../../../components/"

sys.path.append(abs_dir_path + relative_path + "visualization")
sys.path.append(abs_dir_path + relative_path + "state")
sys.path.append(abs_dir_path + relative_path + "vehicle")
sys.path.append(abs_dir_path + relative_path + "obstacle")
sys.path.append(abs_dir_path + relative_path + "sensors")
sys.path.append(abs_dir_path + relative_path + "sensors/lidar")


# import component modules
from global_xy_visualizer import GlobalXYVisualizer
from min_max import MinMax
from time_parameters import TimeParameters
from vehicle_specification import VehicleSpecification
from state import State
from four_wheels_vehicle import FourWheelsVehicle
from obstacle import Obstacle
from obstacle_list import ObstacleList
from sensors import Sensors
from sensor_parameters import SensorParameters
from omni_directional_lidar import OmniDirectionalLidar


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
    lidar = OmniDirectionalLidar(obst_list, SensorParameters(lon_m=spec.wheel_base_m/2)) # lidar instance
    vehicle = FourWheelsVehicle(State(color=spec.color), spec, sensors=Sensors(lidar=lidar)) # set state, spec, lidar as arguments
    vis.add_object(vehicle)

    # plot figure is not shown when executed as unit test
    if not show_plot: vis.not_show_plot()

    # show plot figure
    vis.draw()


# execute main process
if __name__ == "__main__":
    main()

```
![](/src/simulations/perception/lidar_obstacle_sensing/lidar_obstacle_sensing.gif)  