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
        self._hist_plot = None
    
    def _limit_speed(self):
        """
        Function to clamp speed to the stop/max/min limits
        """
        if abs(self.speed_mps) < self.STOP_SPEED_MPS: self.speed_mps = 0.0
        if self.speed_mps > self.MAX_SPEED_MPS: self.speed_mps = self.MAX_SPEED_MPS
        if self.speed_mps < self.MIN_SPEED_MPS: self.speed_mps = self.MIN_SPEED_MPS

    @staticmethod
    def motion_model(state, motion_input, time_s):
        """
        Static function of motion model of vehicle state

        Kinematic bicycle model, discretized over one step of length
        dt = time_s. During the step the heading yaw_t and the acceleration
        a_t are kept constant, so the vehicle moves on a straight segment of
        length v_t * dt + 0.5 * a_t * dt^2 (a plain forward Euler step would
        drop the 0.5 * a_t * dt^2 term):

            x_{t+1}   = x_t + v_t * cos(yaw_t) * dt + 0.5 * a_t * cos(yaw_t) * dt^2
            y_{t+1}   = y_t + v_t * sin(yaw_t) * dt + 0.5 * a_t * sin(yaw_t) * dt^2
            yaw_{t+1} = yaw_t + omega_t * dt
            v_{t+1}   = v_t + a_t * dt

        Because the heading is frozen at yaw_t, the yaw rate omega_t only
        changes yaw for the next step; it does not bend the path within
        this step.

        This model takes the yaw rate omega as input instead of the steering
        angle delta. The two are related by omega = v * tan(delta) / L, where
        L is the wheelbase (Section 3.2 of the book).

        In matrix form, x_{t+1} = A x_t + B u_t with
            state x = [x, y, yaw, v]^T,  input u = [a, omega]^T,  dt = time_s.
        The A matrix holds the state-to-state terms (the v_t * cos/sin * dt
        terms are the 4th column) and B holds the input terms.

        Section 3.3 of the book describes this discretization:
        https://shisatoyano.github.io/AutonomousVehicleControlBeginnersGuide/vehicle_model/index.html

        Reference: R. Rajamani, "Vehicle Dynamics and Control", 2nd ed.,
        Springer, 2012, Ch. 2 (kinematic bicycle model).

        state: Vehicle's state (x, y, yaw, speed) object
        motion_input: Motion input (acceleration, yaw rate) object
        time_s: Time interval per cycle[sec]
        """

        # to fix DeprecationWarning: Conversion of an array with ndim > 0
        # to a scalar is deprecated, and will error in future.
        # Ensure you extract a single element from your array
        # before performing this operation. (Deprecated NumPy 1.25.)
        yaw_rad = state.item(2) # do not extract an element like state[2]

        # A: state transition matrix, x_{t+1} = A x_t + B u_t
        # Rows: x, y, yaw, v. Column 4 multiplies v_t.
        # x and y advance along the frozen heading yaw_t by v_t * dt.
        # yaw and v carry over unchanged (identity) in this model.
        A = np.array([[1, 0, 0, cos(yaw_rad) * time_s],
                      [0, 1, 0, sin(yaw_rad) * time_s],
                      [0, 0, 1, 0],
                      [0, 0, 0, 1]])

        # B: input matrix. Column 1 is acceleration a_t, column 2 is yaw rate omega_t.
        # a_t contributes 0.5 * a_t * dt^2 to x and y (speed integrated exactly
        # under constant acceleration) and a_t * dt to v. omega_t adds
        # omega_t * dt to yaw only; it does not move x or y within the step.
        B = np.array([[(cos(yaw_rad) * time_s**2) / 2, 0],
                      [(sin(yaw_rad) * time_s**2) / 2, 0],
                      [0, time_s],
                      [time_s, 0]])
        
        return A @ state + B @ motion_input

    def update(self, accel_mps2, yaw_rate_rps, time_s):
        """
        Function to update state
        accel_mps2: Acceleration[m/s^2]
        yaw_rate_rps: Yaw rate[rad/s]
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
        
        self._limit_speed()

        self.x_history.append(self.x_m)
        self.y_history.append(self.y_m)
    
    def update_by_localizer(self, state_from_localizer):
        """
        Function to update state by localizer
        state_from_localizer: State estimated by localizer
        """

        self.x_m = state_from_localizer[0, 0]
        self.y_m = state_from_localizer[1, 0]
        self.yaw_rad = state_from_localizer[2, 0]
        self.speed_mps = state_from_localizer[3, 0]

        self._limit_speed()

        self.x_history.append(self.x_m)
        self.y_history.append(self.y_m)

    def x_y_yaw(self):
        """
        Function to get x, y, yaw as array
        """
        
        return np.array([[self.x_m], [self.y_m], [self.yaw_rad]])
    
    def get_x_m(self):
        """
        Function to get x[m]
        """
        
        return self.x_m
    
    def get_y_m(self):
        """
        Function to get y[m]
        """

        return self.y_m
    
    def get_yaw_rad(self):
        """
        Function to get yaw angle[rad]
        """

        return self.yaw_rad
    
    def get_speed_mps(self):
        """
        Function to get speed[m/s]
        """

        return self.speed_mps
    
    def get_speed_kmph(self):
        """
        Function to get speed[km/h]
        """

        return self.speed_mps * 3.6
    
    def draw(self, axes, elems):
        """
        Function to draw x-y history and speed
        """
        
        # The history line is intentionally not appended to elems: it is
        # created once and kept, so set_data can update it every frame
        # instead of re-creating it like the elements in elems.
        if self._hist_plot is None or self._hist_plot.axes is not axes:
            self._hist_plot, = axes.plot(self.x_history, self.y_history, linewidth=0, marker='.', color=self.DRAW_COLOR, zorder=2.5)
        else:
            self._hist_plot.set_data(self.x_history, self.y_history)
            
        elems.append(axes.text(self.x_m, self.y_m + 2, "Speed: " + str(round(self.speed_mps * 3.6, 1)) + "[km/h]", fontsize=10))
