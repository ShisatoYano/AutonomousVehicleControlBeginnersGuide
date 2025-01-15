# 2. Vehicle Model
In this chapter, I'm gonna explain about a vehicle model definition. The vehicle model is defined as a four wheels vehicle based on Kinematics and Geometry. And then, the vehicle is controlled by acceleration input and yaw rate input.  

## 2.1 Vehicle's state
Firstly, an vehicle's state is defined as follow.  

* Position (x, y)[m]
* Yaw angle[rad]
* Speed[m/s]

State class which manages these data is implemented.  
[state.py](/src/components/state/state.py)  

```python
"""
state.py

Author: Shisato Yano
"""

from math import cos, sin
import numpy as np


class State:
    """
    Vehicle state(x, y, yaw, speed) data and logic class
    """

    def __init__(self, x_m=0.0, y_m=0.0, yaw_rad=0.0, speed_mps=0.0, color='k'):
        """
        Constructor
        x_m: Vehicle's position, x[m]
        y_m: Vehicle's position, y[m]
        yaw_rad: Vehicle's yaw angle[rad]
        speed_mps: Vehicle's speed[m/s]
        """

        self.STOP_SPEED_MPS = 0.5 / 3.6 # 0.5[km/h]
        self.MAX_SPEED_MPS = 60 / 3.6 # 60[km/h]
        self.MIN_SPEED_MPS = -10 / 3.6 # -10[km/h]
        self.DRAW_COLOR = color

        self.x_m = x_m
        self.y_m = y_m
        self.yaw_rad = yaw_rad
        self.speed_mps = speed_mps

        self.x_history = [self.x_m]
        self.y_history = [self.y_m]
```

In this code, an initial state including position (x, y), yaw angle and speed is given to a constructor. Additionally, one more argument, "color" is defined for setting the color of position (x, y) plot. And then, the position is stored into 2 member variables, x_history and y_history to record it at each time steps. Finally, 3 constant values, STOP_SPEED_MPS, MAX_SPEED_MPS and MIN_SPEED_MPS are defined to limit a range of the speed computation.  