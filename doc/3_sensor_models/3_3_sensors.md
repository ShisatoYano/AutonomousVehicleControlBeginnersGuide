## 3.3 Sensors
In this section, Sensors class is implemented for managing and processing data of each sensor models. This class can initialize, update and draw the data of each sensor models.  

## 3.3.1 Constructor
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

## 3.3.2 Installation
A function to install each sensors is implemented to compute the global position coordinate on the vehicle. It is computed only when the object of LiDAR class is given to the constructor.  
```python
    def install(self, state):
        """
        Function to calculate each sensor's installation position on vehicle
        state: vehicle's state object
        """

        if self.lidar: self.lidar.install(state)
```

## 3.3.3 Update and Draw
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