"""
ilqr_controller.py

Author: Yuvraj Gupta

Reference:
W. Li and E. Todorov,
"Iterative Linear Quadratic Regulator Design for Nonlinear Biological Movement Systems", ICINCO 2004
Y. Tassa, N. Mansard and E. Todorov,
"Control-Limited Differential Dynamic Programming", ICRA 2014
"""

# import path setting
import sys
from pathlib import Path
from math import tan
import numpy as np
import numpy.typing as npt

abs_dir_path = str(Path(__file__).absolute().parent)
relative_path = "/../../../components/"

sys.path.append(abs_dir_path + relative_path + "common")

# import component modules
from angle_lib import pi_to_pi


class IlqrController:
    """
    Path tracking controller by iLQR (iterative Linear Quadratic Regulator)
    LQR needs a linear model, but the vehicle model is nonlinear.
    iLQR repeats three steps until the cost stops decreasing:
    1. roll out the nonlinear model with the current control sequence
    2. linearize the model around that rollout and solve an LQR problem backward in time
    3. update the control sequence with the LQR gains and a line search
    State z = [x[m], y[m], yaw[rad], speed[m/s]], input u = [acceleration[m/s2], steering[rad]]
    """

    def __init__(self, spec, course=None):
        """
        Constructor
        spec: VehicleSpecification object
        course: Course data and logic object
        """

        self.WHEEL_BASE_M = spec.wheel_base_m

        # horizon
        self.HORIZON_STEPS = 20
        self.DT_SEC = 0.1

        # cost weights of state error [x, y, yaw, speed] and input [accel, steer]
        self.WEIGHT_Q = np.diag([3.0, 3.0, 1.0, 0.5])
        self.WEIGHT_QF = np.diag([5.0, 5.0, 1.0, 0.5])
        self.WEIGHT_R = np.diag([0.1, 1.0])

        # input limits
        self.MAX_ACCEL_MPS2 = 3.0
        self.MAX_STEER_RAD = np.deg2rad(30.0)

        # solver settings
        self.MAX_ITERATION = 5
        self.LINE_SEARCH_ALPHAS = [1.0, 0.5, 0.25, 0.1]
        self.REGULARIZATION = 1e-3
        self.COST_TOLERANCE = 1e-3

        self.course = course
        self.input_sequence = np.zeros((self.HORIZON_STEPS, 2))
        self.predicted_states = None
        self.reference_states = None

        self.target_accel_mps2 = 0.0
        self.target_yaw_rate_rps = 0.0
        self.target_steer_rad = 0.0

    def _dynamics(self, z: npt.NDArray[np.float64], u: npt.NDArray[np.float64]) -> npt.NDArray[np.float64]:
        """
        Private function of the kinematic bicycle model, discretized by Euler method
        z: State vector, shape (4,)
        u: Input vector, shape (2,)
        Return: Next state vector, shape (4,)
        """

        x, y, yaw, v = z
        accel, steer = u
        return np.array([x + v * np.cos(yaw) * self.DT_SEC,
                         y + v * np.sin(yaw) * self.DT_SEC,
                         yaw + v / self.WHEEL_BASE_M * np.tan(steer) * self.DT_SEC,
                         v + accel * self.DT_SEC])

    def _jacobians(self, z: npt.NDArray[np.float64],
                   u: npt.NDArray[np.float64]) -> tuple[npt.NDArray[np.float64], npt.NDArray[np.float64]]:
        """
        Private function to linearize the model: z[k+1] ~ A z[k] + B u[k]
        z: State vector, shape (4,)
        u: Input vector, shape (2,)
        Return: A shape (4, 4), B shape (4, 2)
        """

        _, _, yaw, v = z
        _, steer = u
        dt = self.DT_SEC

        A = np.eye(4)
        A[0, 2] = -v * np.sin(yaw) * dt
        A[0, 3] = np.cos(yaw) * dt
        A[1, 2] = v * np.cos(yaw) * dt
        A[1, 3] = np.sin(yaw) * dt
        A[2, 3] = np.tan(steer) / self.WHEEL_BASE_M * dt

        B = np.zeros((4, 2))
        B[2, 1] = v / (self.WHEEL_BASE_M * np.cos(steer) ** 2) * dt
        B[3, 0] = dt
        return A, B

    def _state_error(self, z: npt.NDArray[np.float64], z_ref: npt.NDArray[np.float64]) -> npt.NDArray[np.float64]:
        """
        Private function to calculate state error with the yaw error wrapped into -pi..pi
        """

        error = z - z_ref
        error[2] = pi_to_pi(error[2])
        return error

    def _rollout(self, z0: npt.NDArray[np.float64], inputs: npt.NDArray[np.float64]) -> npt.NDArray[np.float64]:
        """
        Private function to simulate the model with an input sequence
        Return: State sequence, shape (N + 1, 4)
        """

        states = np.zeros((self.HORIZON_STEPS + 1, 4))
        states[0] = z0
        for k in range(self.HORIZON_STEPS):
            states[k + 1] = self._dynamics(states[k], inputs[k])
        return states

    def _total_cost(self, states: npt.NDArray[np.float64], inputs: npt.NDArray[np.float64]) -> float:
        """
        Private function to calculate the cost of a trajectory
        """

        cost = 0.0
        for k in range(self.HORIZON_STEPS):
            e = self._state_error(states[k], self.reference_states[k])
            cost += e @ self.WEIGHT_Q @ e + inputs[k] @ self.WEIGHT_R @ inputs[k]
        e = self._state_error(states[-1], self.reference_states[-1])
        return cost + e @ self.WEIGHT_QF @ e

    def _backward_pass(self, states: npt.NDArray[np.float64],
                       inputs: npt.NDArray[np.float64]) -> tuple[list, list]:
        """
        Private function to calculate feedforward terms k and feedback gains K
        by solving the LQR problem of the linearized model backward in time
        """

        e = self._state_error(states[-1], self.reference_states[-1])
        v_x = self.WEIGHT_QF @ e  # gradient of the value function
        v_xx = self.WEIGHT_QF.copy()  # hessian of the value function

        k_list, gain_list = [None] * self.HORIZON_STEPS, [None] * self.HORIZON_STEPS
        for k in reversed(range(self.HORIZON_STEPS)):
            A, B = self._jacobians(states[k], inputs[k])
            e = self._state_error(states[k], self.reference_states[k])

            q_x = self.WEIGHT_Q @ e + A.T @ v_x
            q_u = self.WEIGHT_R @ inputs[k] + B.T @ v_x
            q_xx = self.WEIGHT_Q + A.T @ v_xx @ A
            q_uu = self.WEIGHT_R + B.T @ v_xx @ B + self.REGULARIZATION * np.eye(2)
            q_ux = B.T @ v_xx @ A

            q_uu_inv = np.linalg.inv(q_uu)
            k_ff = -q_uu_inv @ q_u
            gain = -q_uu_inv @ q_ux

            v_x = q_x + gain.T @ q_uu @ k_ff + gain.T @ q_u + q_ux.T @ k_ff
            v_xx = q_xx + gain.T @ q_uu @ gain + gain.T @ q_ux + q_ux.T @ gain
            v_xx = 0.5 * (v_xx + v_xx.T)

            k_list[k], gain_list[k] = k_ff, gain
        return k_list, gain_list

    def _forward_pass(self, states: npt.NDArray[np.float64], inputs: npt.NDArray[np.float64],
                      k_list: list, gain_list: list, alpha: float) -> tuple[npt.NDArray[np.float64], npt.NDArray[np.float64]]:
        """
        Private function to apply the new control law to the nonlinear model
        u_new = u + alpha * k + K (z_new - z)
        """

        new_states = np.zeros_like(states)
        new_inputs = np.zeros_like(inputs)
        new_states[0] = states[0]
        for k in range(self.HORIZON_STEPS):
            dz = self._state_error(new_states[k], states[k])
            u = inputs[k] + alpha * k_list[k] + gain_list[k] @ dz
            u[0] = np.clip(u[0], -self.MAX_ACCEL_MPS2, self.MAX_ACCEL_MPS2)
            u[1] = np.clip(u[1], -self.MAX_STEER_RAD, self.MAX_STEER_RAD)
            new_inputs[k] = u
            new_states[k + 1] = self._dynamics(new_states[k], u)
        return new_states, new_inputs

    def _make_reference(self, state) -> None:
        """
        Private function to sample reference states on the course for the horizon
        state: Vehicle's state object
        """

        index = self.course.search_nearest_point_index(state)
        last_index = len(self.course.x_array) - 1
        point_interval_m = np.hypot(self.course.point_x_m(1) - self.course.point_x_m(0),
                                    self.course.point_y_m(1) - self.course.point_y_m(0))
        ref = np.zeros((self.HORIZON_STEPS + 1, 4))
        travel_m = 0.0
        for k in range(self.HORIZON_STEPS + 1):
            # move along the course by the distance driven at the target speed
            i = min(index + int(round(travel_m / point_interval_m)), last_index)
            ref[k] = [self.course.point_x_m(i), self.course.point_y_m(i),
                      self.course.point_yaw_rad(i), self.course.point_speed_mps(i)]
            travel_m += abs(ref[k, 3]) * self.DT_SEC
        self.reference_states = ref

    def _solve(self, z0: npt.NDArray[np.float64]) -> None:
        """
        Private function of the iLQR iterations
        z0: Current state vector, shape (4,)
        """

        inputs = self.input_sequence
        states = self._rollout(z0, inputs)
        cost = self._total_cost(states, inputs)

        for _ in range(self.MAX_ITERATION):
            k_list, gain_list = self._backward_pass(states, inputs)

            improved = False
            for alpha in self.LINE_SEARCH_ALPHAS:
                new_states, new_inputs = self._forward_pass(states, inputs, k_list, gain_list, alpha)
                new_cost = self._total_cost(new_states, new_inputs)
                if new_cost < cost:
                    improved = True
                    break

            if not improved: break

            converged = (cost - new_cost) < self.COST_TOLERANCE
            states, inputs, cost = new_states, new_inputs, new_cost
            if converged: break

        self.input_sequence = inputs
        self.predicted_states = states

    def update(self, state, time_s: float) -> None:
        """
        Function to update data for path tracking
        state: Vehicle's state object
        time_s: Simulation interval time[sec]
        """

        if not self.course: return

        self._make_reference(state)
        z0 = np.array([state.get_x_m(), state.get_y_m(), state.get_yaw_rad(), state.get_speed_mps()])
        self._solve(z0)

        self.target_accel_mps2 = self.input_sequence[0, 0]
        self.target_steer_rad = self.input_sequence[0, 1]
        self.target_yaw_rate_rps = state.get_speed_mps() * tan(self.target_steer_rad) / self.WHEEL_BASE_M

        # warm start: shift the input sequence by one step for the next cycle
        self.input_sequence = np.vstack((self.input_sequence[1:], self.input_sequence[-1:]))

    def get_target_accel_mps2(self) -> float:
        """
        Function to get acceleration input[m/s2]
        """

        return self.target_accel_mps2

    def get_target_yaw_rate_rps(self) -> float:
        """
        Function to get yaw rate input[rad/s]
        """

        return self.target_yaw_rate_rps

    def get_target_steer_rad(self) -> float:
        """
        Function to get steering angle input[rad]
        """

        return self.target_steer_rad

    def draw(self, axes, elems) -> None:
        """
        Function to draw the predicted trajectory over the horizon
        axes: Axes object of figure
        elems: List of plot objects
        """

        if self.predicted_states is None: return
        predict_plot, = axes.plot(self.predicted_states[:, 0], self.predicted_states[:, 1],
                                  lw=2.0, color='m', marker='.', label="iLQR prediction")
        elems.append(predict_plot)
