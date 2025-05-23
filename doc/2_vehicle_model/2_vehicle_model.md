# 2. Vehicle Model
In this chapter, I'm gonna explain about a vehicle model definition. The vehicle model is defined as a four wheels vehicle based on Kinematics and Geometry. And then, the vehicle is controlled by acceleration input and yaw rate input.  

## 2.1 Vehicle's state
A vehicle's state is defined as follow.  

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

## 2.2 Vehicle's motion and State equation
I define the vehicle's motion and a state equation in this section. The vehicle's motion is defined as constant acceleration linear motion model. Then, an input given to the vehicle is acceleration[m/s2] and yaw rate[rad/s]. The vehicle's state can be updated with the input based on the motion model. The positive direction of the vehicle's yaw angle is left direction. This model can be implemented as State class's member methos as follow.  

```python
    @staticmethod
    def motion_model(state, input, time_s):
        """
        Static function of motion model of vehicle state
        state: Vehicle's state (x, y, yaw, speed) object
        input: Motion input (acceleration, yaw rate) object
        time_s: Time interval per cycle[sec]
        """

        # to fix DeprecationWarning: Conversion of an array with ndim > 0 
        # to a scalar is deprecated, and will error in future. 
        # Ensure you extract a single element from your array 
        # before performing this operation. (Deprecated NumPy 1.25.)
        yaw_rad = state.item(2) # do not extract an element like state[2]
        
        A = np.array([[1, 0, 0, cos(yaw_rad) * time_s],
                      [0, 1, 0, sin(yaw_rad) * time_s],
                      [0, 0, 1, 0],
                      [0, 0, 0, 1]])
        
        B = np.array([[(cos(yaw_rad) * time_s**2) / 2, 0],
                      [(sin(yaw_rad) * time_s**2) / 2, 0],
                      [0, time_s],
                      [time_s, 0]])
        
        return A @ state + B @ input
```

This method is defined as a static method. When you want to use this method, you don't need to generate the State class's object. 2 matrix A and B in this code is to represent multiple state variables as a state equation.  

## 2.3 Updating vehicle's state
I implement a member method of State class, "update" to compute the vehicle's state at the next time step. The input of acceleration, yaw rate and an interval time per cycle[sec] are given as arguments.  

```python
    def update(self, accel_mps2, yaw_rate_rps, time_s):
        """
        Function to update state
        accel_mps2: Acceleration[m/s^2]
        steer_rad: Steering angle[rad]
        time_s: Time interval per cycle[sec]
        """

        last_state = np.array([[self.x_m],
                               [self.y_m],
                               [self.yaw_rad],
                               [self.speed_mps]])
        
        next_input = np.array([[accel_mps2],
                               [yaw_rate_rps]])
        
        next_state = self.motion_model(last_state, next_input, time_s)

        self.x_m = next_state[0, 0]
        self.y_m = next_state[1, 0]
        self.yaw_rad = next_state[2, 0]
        self.speed_mps = next_state[3, 0]
        
        if abs(self.speed_mps) < self.STOP_SPEED_MPS: self.speed_mps = 0.0
        if self.speed_mps > self.MAX_SPEED_MPS: self.speed_mps = self.MAX_SPEED_MPS
        if self.speed_mps < self.MIN_SPEED_MPS: self.speed_mps = self.MIN_SPEED_MPS

        self.x_history.append(self.x_m)
        self.y_history.append(self.y_m)
```

In this code, a new state at the next time step can be computed with the method, "motion_model". If the updated speed was lower than STOP_SPEED_MPS, the speed would be set to 0 to make the vehicle stopped. And then, the speed is limited between MAX_SPEED_MPS and MIN_SPEED_MPS. Finally, the updated position (x, y) is stored to those list of history.  

## 2.4 Vehicle's parts class
This section provides the explanation of the vehicle's parts class. The vehicle is separated into the following multiple parts and those drawing classes are implemented.  

* Body
* Chasis
* Front axle
* Rear axle
* Front Right/Left tire
* Rear Right/Left tire

### 2.4.1 Drawing method
Implementing a member method of State class, "draw" for visualization. In this code, Those lists of x, y history are plot and the current speed is also visualized.  

```python
    def draw(self, axes, elems):
        """
        Function to draw x-y history and speed
        """
        
        hist_plot, = axes.plot(self.x_history, self.y_history, linewidth=0, marker='.', color=self.DRAW_COLOR)
        elems.append(hist_plot)

        elems.append(axes.text(self.x_m, self.y_m + 2, "Speed: " + str(round(self.speed_mps * 3.6, 1)) + "[km/h]", fontsize=10))
```

### 2.4.2 X-Y Array class
The vehicle's drawing is represented as x-y 2D array. So, I implement X-Y Array class as follow.  
[xy_array.py](/src/components/array/xy_array.py)  

```python
"""
xy_array.py

Author: Shisato Yano
"""

from math import sin, cos
import numpy as np


class XYArray:
    """
    X-Y 2D array data and logic class
    """

    def __init__(self, data):
        """
        Constructor
        data: np.array([[x1, x2,..., xn], [y1, y2,..., yn]])
        """

        self.data = data
    
    def homogeneous_transformation(self, x, y, angle_rad):
        """
        Function for homogeneous transformation
        x: Amount of x-axis translation
        y: Amount of y-axis translation
        angle_rad: Rotation angle[rad]
        Return transformed XYArray object
        """
        
        angle_cos = cos(angle_rad)
        angle_sin = sin(angle_rad)

        rotation_matrix = np.array([[angle_cos, -angle_sin],
                                    [angle_sin, angle_cos]])
        
        rotated_data = rotation_matrix @ self.data

        translated_data = rotated_data + np.ones(rotated_data.shape) * np.array([[x], [y]])

        return XYArray(translated_data)
```

In this code, a given data to Constructor is x-y 2D array. Then, the data can be transformed based on the vehicle's position (x, y) and yaw angle. After the data was transformed, a new XYArray object with the transformed data is returned.  

### 2.4.3 Specification class
Implementing Vehicle's specification class as follow. This class is used to manage each parameters for the vehicle's control.  
[vehicle_specification.py](/src/components/vehicle/vehicle_specification.py)  

```python
"""
vehicle_specification.py

Author: Shisato Yano
"""

import sys
from pathlib import Path

sys.path.append(str(Path(__file__).absolute().parent) + "/../visualization")
from min_max import MinMax

class VehicleSpecification:
    """
    Vehicle Specification parameters class
    """
    
    def __init__(self, f_len_m=2.0, r_len_m=0.0, tire_r_m=0.3,
                 tire_w_m=0.12, axle_half_m=0.5, color='k', 
                 line_w=1.0, line_type='-', area_size=10.0,
                 x_lim=MinMax(-30, 30), y_lim=MinMax(-30, 30),
                 max_accel_mps2=3.0):
        """
        Constructor
        f_len_m: length[m] from origin to center of front axle
        r_len_m: length[m] from origin to center of rear axle
        tire_r_m: tire's radius[m]
        tire_w_m: tire's half of width[m]
        color: vehicle's color
        line_w: plot line's width
        line_type: plot line's type
        area_size: plot area size[m]
        x_lim: min/max values of x-axis
        y_lim: min/max values of y-axis
        max_accel_mps2: maximum acceleration/deceleration[m/s2]
        """

        self.f_len_m = f_len_m
        self.f_edge_m = self.f_len_m + 0.5

        self.r_len_m = r_len_m
        self.r_edge_m = self.r_len_m + 0.5

        self.tread_m = 0.25 * (1.0 + self.f_len_m + self.r_len_m)
        self.width_m = 1.0 * self.tread_m
        self.wheel_base_m = self.f_len_m + self.r_len_m
        
        self.tire_r_m = tire_r_m
        self.tire_w_m = tire_w_m
        self.axle_half_m = axle_half_m

        self.color = color
        self.line_w = line_w
        self.line_type = line_type
        self.area_size = area_size

        self.x_lim = x_lim
        self.y_lim = y_lim

        self.max_accel_mps2 = max_accel_mps2
```

The vehicle has the following parameters as specification. The vehicle is drawn according to the parameters.  

* Length from origin to center of front axle[m]
* Length from origin to center of rear axle[m]
* Length from origin to edge of front body[m]
* Length from origin to edge of rear body[m]
* Length of Tread[m]
* Body's width[m]
* Length of wheel base[m]
* Tire's radius[m]
* Tire's width[m]
* Half of axle length[m]
* Drawing color
* Type of line
* Size of zoom area around vehicle[m]
* Limitation in x axis[m]
* Limitation in y axis[m]
* Maximum acceleration[m/s2]

### 2.4.4 Body class
Vehicle body class is implemented as follow. This class is used for drawing the vehicle's body according to the specification.  
[body.py](/src/components/vehicle/body.py)  

```python
"""
body.py

Author: Shisato Yano
"""

import numpy as np
import sys
from pathlib import Path

sys.path.append(str(Path(__file__).absolute().parent) + "/../array")
from xy_array import XYArray


class Body:
    """
    Vehicle Body class
    """

    def __init__(self, spec):
        """
        Constructor
        spec: object of VehicleSpecification class
        """

        self.spec = spec

        contour = np.array([[self.spec.f_edge_m, -self.spec.r_edge_m, -self.spec.r_edge_m, self.spec.f_edge_m, self.spec.f_edge_m],
                            [self.spec.width_m, self.spec.width_m, -self.spec.width_m, -self.spec.width_m, self.spec.width_m]])
        self.array = XYArray(contour)

    def draw(self, axes, pose, elems):
        """
        Function to plot vehicle's body lines
        axes: Axes object of figure
        pose: Vehicle's pose vector
        elems: List of plot objects
        """

        transformed_array = self.array.homogeneous_transformation(pose[0, 0], pose[1, 0], pose[2, 0])
        body_plot, = axes.plot(transformed_array.get_x_data(), transformed_array.get_y_data(), 
                               lw=self.spec.line_w, color=self.spec.color, ls=self.spec.line_type)
        elems.append(body_plot)
```

### 2.4.5 Chassis class
Vehicle chassis class is implemented as follow. This class is used for drawing the vehicle's chassis according to the specification.  
[chassis.py](/src/components/vehicle/chassis.py)  

```python
"""
chassis.py

Author: Shisato Yano
"""

import numpy as np
import sys
from pathlib import Path

sys.path.append(str(Path(__file__).absolute().parent) + "/../array")
from xy_array import XYArray


class Chassis:
    """
    Vehicle Chassis class
    """

    def __init__(self, spec):
        """
        Constructor
        spec: object of VehicleSpecification class
        """
        
        self.spec = spec

        contour = np.array([[self.spec.f_len_m, -self.spec.r_len_m],
                            [0.0, 0.0]])
        self.array = XYArray(contour)
    
    def draw(self, axes, pose, elems):
        """
        Function to plot vehicle's chassis lines
        axes: Axes object of figure
        pose: Vehicle's pose vector
        elems: List of plot objects
        """

        transformed_array = self.array.homogeneous_transformation(pose[0, 0], pose[1, 0], pose[2, 0])
        chassis_plot, = axes.plot(transformed_array.get_x_data(), transformed_array.get_y_data(), 
                                  lw=self.spec.line_w, color=self.spec.color, ls=self.spec.line_type)
        elems.append(chassis_plot)
```

### 2.4.6 Front axle class
Vehicle front axle class is implemented as follow. This class is used for drawing the vehicle's front axle according to the specification.  
[front_axle.py](/src/components/vehicle/front_axle.py)  

```python
"""
front_axle.py

Author: Shisato Yano
"""

import numpy as np
import sys
from pathlib import Path

sys.path.append(str(Path(__file__).absolute().parent) + "/../array")
from xy_array import XYArray


class FrontAxle:
    """
    Vehicle Front Axle class
    """

    def __init__(self, spec):
        """
        Constructor
        spec: object of VehicleSpecification class
        """

        self.spec = spec
        self.offset_x_m = spec.f_len_m
        self.offset_y_m = spec.axle_half_m

        contour = np.array([[0.0, 0.0], 
                            [self.offset_y_m, -self.offset_y_m]])
        self.array = XYArray(contour)
    
    def draw(self, axes, pose, elems):
        """
        Function to plot vehicle's front axle's lines
        axes: Axes object of figure
        pose: Vehicle's pose vector
        elems: List of plot objects
        """
        
        translated_array = self.array.homogeneous_transformation(self.offset_x_m, 0.0, 0.0)
        transformed_array = translated_array.homogeneous_transformation(pose[0, 0], pose[1, 0], pose[2, 0])
        axle_plot, = axes.plot(transformed_array.get_x_data(), transformed_array.get_y_data(), 
                               lw=self.spec.line_w, color=self.spec.color, ls=self.spec.line_type)
        elems.append(axle_plot)
```

### 2.4.7 Rear axle class
Vehicle rear axle class is implemented as follow. This class is used for drawing the vehicle's rear axle according to the specification.  
[rear_axle.py](/src/components/vehicle/rear_axle.py)  

```python
"""
rear_axle.py

Author: Shisato Yano
"""

import numpy as np
import sys
from pathlib import Path

sys.path.append(str(Path(__file__).absolute().parent) + "/../array")
from xy_array import XYArray


class RearAxle:
    """
    Vehicle Rear Axle class
    """

    def __init__(self, spec):
        """
        Constructor
        spec: object of VehicleSpecification class
        """

        self.spec = spec
        self.offset_x_m = spec.r_len_m
        self.offset_y_m = spec.axle_half_m

        contour = np.array([[0.0, 0.0], 
                            [self.offset_y_m, -self.offset_y_m]])
        self.array = XYArray(contour)
    
    def draw(self, axes, pose, elems):
        """
        Function to plot vehicle's rear axle's lines
        axes: Axes object of figure
        pose: Vehicle's pose vector
        elems: List of plot objects
        """
        
        translated_array = self.array.homogeneous_transformation(self.offset_x_m, 0.0, 0.0)
        transformed_array = translated_array.homogeneous_transformation(pose[0, 0], pose[1, 0], pose[2, 0])
        axle_plot, = axes.plot(transformed_array.get_x_data(), transformed_array.get_y_data(), 
                               lw=self.spec.line_w, color=self.spec.color, ls=self.spec.line_type)
        elems.append(axle_plot)
```

### 2.4.8 Front left tire class
Vehicle front left tire class is implemented as follow. This class is used for drawing the vehicle's front left tire according to the specification.  
[front_left_tire.py](/src/components/vehicle/front_left_tire.py)  

```python
"""
front_left_tire.py

Author: Shisato Yano
"""

import numpy as np
import sys
from pathlib import Path

sys.path.append(str(Path(__file__).absolute().parent) + "/../array")
from xy_array import XYArray


class FrontLeftTire:
    """
    Vehicle Front Left Tire class
    """

    def __init__(self, spec):
        """
        Constructor
        spec: object of VehicleSpecification class
        """
        
        self.spec = spec
        self.offset_x_m = spec.f_len_m
        self.offset_y_m = spec.axle_half_m

        contour = np.array([[self.spec.tire_r_m, -self.spec.tire_r_m, -self.spec.tire_r_m, self.spec.tire_r_m, self.spec.tire_r_m],
                            [self.spec.tire_w_m, self.spec.tire_w_m, -self.spec.tire_w_m, -self.spec.tire_w_m, self.spec.tire_w_m]])
        self.array = XYArray(contour)
    
    def draw(self, axes, pose, steer_rad, elems):
        """
        Function to plot vehicle's front left tire's lines
        axes: Axes object of figure
        pose: Vehicle's pose vector
        steer_rad: Tire's steering angle[rad]
        elems: List of plot objects
        """

        translated_array = self.array.homogeneous_transformation(self.offset_x_m, self.offset_y_m, steer_rad)
        transformed_array = translated_array.homogeneous_transformation(pose[0, 0], pose[1, 0], pose[2, 0])
        tire_plot, = axes.plot(transformed_array.get_x_data(), transformed_array.get_y_data(), 
                               lw=self.spec.line_w, color=self.spec.color, ls=self.spec.line_type)
        elems.append(tire_plot)
```

### 2.4.9 Front right tire class
Vehicle front right tire class is implemented as follow. This class is used for drawing the vehicle's front right tire according to the specification.  
[front_right_tire.py](/src/components/vehicle/front_right_tire.py)  

```python
"""
front_right_tire.py

Author: Shisato Yano
"""

import numpy as np
import sys
from pathlib import Path

sys.path.append(str(Path(__file__).absolute().parent) + "/../array")
from xy_array import XYArray


class FrontRightTire:
    """
    Vehicle Front Right Tire class
    """

    def __init__(self, spec):
        """
        Constructor
        spec: object of VehicleSpecification class
        """
        
        self.spec = spec
        self.offset_x_m = spec.f_len_m
        self.offset_y_m = -spec.axle_half_m

        contour = np.array([[self.spec.tire_r_m, -self.spec.tire_r_m, -self.spec.tire_r_m, self.spec.tire_r_m, self.spec.tire_r_m],
                            [self.spec.tire_w_m, self.spec.tire_w_m, -self.spec.tire_w_m, -self.spec.tire_w_m, self.spec.tire_w_m]])
        self.array = XYArray(contour)
    
    def draw(self, axes, pose, steer_rad, elems):
        """
        Function to plot vehicle's front right tire's lines
        axes: Axes object of figure
        pose: Vehicle's pose vector
        steer_rad: Tire's steering angle[rad]
        elems: List of plot objects
        """
        
        translated_array = self.array.homogeneous_transformation(self.offset_x_m, self.offset_y_m, steer_rad)
        transformed_array = translated_array.homogeneous_transformation(pose[0, 0], pose[1, 0], pose[2, 0])
        tire_plot, = axes.plot(transformed_array.get_x_data(), transformed_array.get_y_data(), 
                               lw=self.spec.line_w, color=self.spec.color, ls=self.spec.line_type)
        elems.append(tire_plot)
```

### 2.4.10 Rear left tire class
Vehicle rear left tire class is implemented as follow. This class is used for drawing the vehicle's rear left tire according to the specification.  
[rear_left_tire.py](/src/components/vehicle/rear_left_tire.py)  

```python
"""
rear_left_tire.py

Author: Shisato Yano
"""

import numpy as np
import sys
from pathlib import Path

sys.path.append(str(Path(__file__).absolute().parent) + "/../array")
from xy_array import XYArray


class RearLeftTire:
    """
    Vehicle Rear Left Tire class
    """

    def __init__(self, spec):
        """
        Constructor
        spec: object of VehicleSpecification class
        """
        
        self.spec = spec
        self.offset_x_m = -spec.r_len_m
        self.offset_y_m = spec.axle_half_m

        contour = np.array([[self.spec.tire_r_m, -self.spec.tire_r_m, -self.spec.tire_r_m, self.spec.tire_r_m, self.spec.tire_r_m],
                            [self.spec.tire_w_m, self.spec.tire_w_m, -self.spec.tire_w_m, -self.spec.tire_w_m, self.spec.tire_w_m]])
        self.array = XYArray(contour)
    
    def draw(self, axes, pose, elems):
        """
        Function to plot vehicle's rear left tire's lines
        axes: Axes object of figure
        pose: Vehicle's pose vector
        elems: List of plot objects
        """
        
        translated_array = self.array.homogeneous_transformation(self.offset_x_m, self.offset_y_m, 0.0)
        transformed_array = translated_array.homogeneous_transformation(pose[0, 0], pose[1, 0], pose[2, 0])
        tire_plot, = axes.plot(transformed_array.get_x_data(), transformed_array.get_y_data(), 
                               lw=self.spec.line_w, color=self.spec.color, ls=self.spec.line_type)
        elems.append(tire_plot)
```

### 2.4.11 Rear right tire class
Vehicle rear left tire class is implemented as follow. This class is used for drawing the vehicle's rear left tire according to the specification.  
[rear_right_tire.py](/src/components/vehicle/rear_right_tire.py)  

```python
"""
rear_right_tire.py

Author: Shisato Yano
"""

import numpy as np
import sys
from pathlib import Path

sys.path.append(str(Path(__file__).absolute().parent) + "/../array")
from xy_array import XYArray


class RearRightTire:
    """
    Vehicle Rear Right Tire class
    """

    def __init__(self, spec):
        """
        Constructor
        spec: object of VehicleSpecification class
        """
        
        self.spec = spec
        self.offset_x_m = -spec.r_len_m
        self.offset_y_m = -spec.axle_half_m

        contour = np.array([[self.spec.tire_r_m, -self.spec.tire_r_m, -self.spec.tire_r_m, self.spec.tire_r_m, self.spec.tire_r_m],
                            [self.spec.tire_w_m, self.spec.tire_w_m, -self.spec.tire_w_m, -self.spec.tire_w_m, self.spec.tire_w_m]])
        self.array = XYArray(contour)
    
    def draw(self, axes, pose, elems):
        """
        Function to plot vehicle's rear right tire's lines
        axes: Axes object of figure
        pose: Vehicle's pose vector
        elems: List of plot objects
        """
        
        translated_array = self.array.homogeneous_transformation(self.offset_x_m, self.offset_y_m, 0.0)
        transformed_array = translated_array.homogeneous_transformation(pose[0, 0], pose[1, 0], pose[2, 0])
        tire_plot, = axes.plot(transformed_array.get_x_data(), transformed_array.get_y_data(), 
                               lw=self.spec.line_w, color=self.spec.color, ls=self.spec.line_type)
        elems.append(tire_plot)
```

### 2.4.12 Four wheels vehicle class
FourWheelsVehicle class is implemented by importing the above each parts class.  
[four_wheels_vehicle.py](/src/components/vehicle/four_wheels_vehicle.py)  

```python
"""
four_wheels_vehicle.py

Author: Shisato Yano
"""

from body import Body
from chassis import Chassis
from front_left_tire import FrontLeftTire
from front_right_tire import FrontRightTire
from rear_left_tire import RearLeftTire
from rear_right_tire import RearRightTire
from front_axle import FrontAxle
from rear_axle import RearAxle


class FourWheelsVehicle:
    """
    Four Wheels Vehicle model class
    """

    def __init__(self, state, spec, show_zoom=True):
        """
        Constructor
        state: Vehicle's state object
        spec: Vehicle's specification object
        show_zoom: Flag for zoom around vehicle
        """
        
        self.state = state

        self.spec = spec
        self.body = Body(spec)
        self.chassis = Chassis(spec)
        self.front_left_tire = FrontLeftTire(spec)
        self.front_right_tire = FrontRightTire(spec)
        self.rear_left_tire = RearLeftTire(spec)
        self.rear_right_tire = RearRightTire(spec)
        self.front_axle = FrontAxle(spec)
        self.rear_axle = RearAxle(spec)

        self.show_zoom = show_zoom

    def update(self, time_s):
        """
        Function to update each member objects
        time_s: Simulation interval time[sec]
        """

        self.state.update(0.0, 0.0, time_s)
    
    def draw(self, axes, elems):
        """
        Function to draw each member object's data
        axes: Axes object of figure
        elems: List of plot object
        """

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

## 2.5 Visualization
An object of the above FourWheelsVehicle class can be added to the world by executing the following sample programs.  

### 2.5.1 Without zoom
This program can visualize the vehicle and whole world in global coordinate system.  
[visualize_vehicle.py](/doc/2_vehicle_model/visualize_vehicle.py)  

```python
"""
visualize_vehicle.py

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


# import component modules
from global_xy_visualizer import GlobalXYVisualizer
from min_max import MinMax
from time_parameters import TimeParameters
from vehicle_specification import VehicleSpecification
from state import State
from four_wheels_vehicle import FourWheelsVehicle


# flag to show plot figure
# when executed as unit test, this flag is set as false
show_plot = True


def main():
    """
    Main process function
    """
    
    # set simulation parameters
    x_lim, y_lim = MinMax(-30, 30), MinMax(-30, 30)
    vis = GlobalXYVisualizer(x_lim, y_lim, TimeParameters(span_sec=20), show_zoom=False)

    # create vehicle
    spec = VehicleSpecification() # specification
    vehicle = FourWheelsVehicle(State(), spec, show_zoom=False) # set state and spec

    # add objects here
    vis.add_object(vehicle)

    # plot figure is not shown when executed as unit test
    if not show_plot: vis.not_show_plot()

    # show plot figure
    vis.draw()


# execute main process
if __name__ == "__main__":
    main()
```

You can see the following animation by executing this program.  
![](/doc/2_vehicle_model/visualize_vehicle.gif)  

### 2.5.2 With zoom
This program can visualize the vehicle within an limited area in global coordinate system. The size of limited area can be set as parameters.  
[visualize_vehicle_zoom.py](/doc/2_vehicle_model/visualize_vehicle_zoom.py)  

```python
"""
visualize_vehicle_zoom.py

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


# import component modules
from global_xy_visualizer import GlobalXYVisualizer
from min_max import MinMax
from time_parameters import TimeParameters
from vehicle_specification import VehicleSpecification
from state import State
from four_wheels_vehicle import FourWheelsVehicle


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

    # create vehicle
    spec = VehicleSpecification() # specification
    vehicle = FourWheelsVehicle(State(), spec) # set state and spec

    # add objects here
    vis.add_object(vehicle)

    # plot figure is not shown when executed as unit test
    if not show_plot: vis.not_show_plot()

    # show plot figure
    vis.draw()


# execute main process
if __name__ == "__main__":
    main()
```

You can see the following animation by executing this program. In this animation, the size of limited area is 10m x 10m. This size is defined as default values of VehicleSpecification class's parameters.  
![](/doc/2_vehicle_model/visualize_vehicle_zoom.gif)  