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

## 2.4 Visualization
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