"""
polynomials.py

Author: Yuvraj Gupta
"""

import numpy as np
import numpy.typing as npt


class QuinticPolynomial:
    """
    Quintic (5th order) polynomial in time
    p(t) = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5
    Used for the lateral motion because position, velocity and
    acceleration are fixed at both the start and the end
    """

    def __init__(self, p0: float, v0: float, a0: float,
                 p1: float, v1: float, a1: float, time_s: float):
        """
        Constructor
        p0, v0, a0: Position, velocity and acceleration at t = 0
        p1, v1, a1: Position, velocity and acceleration at t = time_s
        time_s: Duration of the polynomial[sec]
        """

        # the first three coefficients come directly from the start conditions
        self.a0 = p0
        self.a1 = v0
        self.a2 = a0 / 2.0

        # the last three coefficients are solved from the end conditions
        t = time_s
        mat_a = np.array([[t ** 3, t ** 4, t ** 5],
                          [3 * t ** 2, 4 * t ** 3, 5 * t ** 4],
                          [6 * t, 12 * t ** 2, 20 * t ** 3]])
        vec_b = np.array([p1 - self.a0 - self.a1 * t - self.a2 * t ** 2,
                          v1 - self.a1 - 2 * self.a2 * t,
                          a1 - 2 * self.a2])
        self.a3, self.a4, self.a5 = np.linalg.solve(mat_a, vec_b)

    def position(self, t: npt.NDArray[np.float64]) -> npt.NDArray[np.float64]:
        """
        Function to calculate position at time t
        t: Array of times[sec], shape (N,)
        """

        return self.a0 + self.a1 * t + self.a2 * t ** 2 + \
            self.a3 * t ** 3 + self.a4 * t ** 4 + self.a5 * t ** 5

    def velocity(self, t: npt.NDArray[np.float64]) -> npt.NDArray[np.float64]:
        """
        Function to calculate first derivative at time t
        t: Array of times[sec], shape (N,)
        """

        return self.a1 + 2 * self.a2 * t + 3 * self.a3 * t ** 2 + \
            4 * self.a4 * t ** 3 + 5 * self.a5 * t ** 4

    def acceleration(self, t: npt.NDArray[np.float64]) -> npt.NDArray[np.float64]:
        """
        Function to calculate second derivative at time t
        t: Array of times[sec], shape (N,)
        """

        return 2 * self.a2 + 6 * self.a3 * t + 12 * self.a4 * t ** 2 + 20 * self.a5 * t ** 3

    def jerk(self, t: npt.NDArray[np.float64]) -> npt.NDArray[np.float64]:
        """
        Function to calculate third derivative at time t
        t: Array of times[sec], shape (N,)
        """

        return 6 * self.a3 + 24 * self.a4 * t + 60 * self.a5 * t ** 2


class QuarticPolynomial:
    """
    Quartic (4th order) polynomial in time
    p(t) = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4
    Used for the longitudinal motion because only the end velocity and
    acceleration are fixed (the end position is free, "velocity keeping")
    """

    def __init__(self, p0: float, v0: float, a0: float,
                 v1: float, a1: float, time_s: float):
        """
        Constructor
        p0, v0, a0: Position, velocity and acceleration at t = 0
        v1, a1: Velocity and acceleration at t = time_s
        time_s: Duration of the polynomial[sec]
        """

        self.a0 = p0
        self.a1 = v0
        self.a2 = a0 / 2.0

        t = time_s
        mat_a = np.array([[3 * t ** 2, 4 * t ** 3],
                          [6 * t, 12 * t ** 2]])
        vec_b = np.array([v1 - self.a1 - 2 * self.a2 * t,
                          a1 - 2 * self.a2])
        self.a3, self.a4 = np.linalg.solve(mat_a, vec_b)

    def position(self, t: npt.NDArray[np.float64]) -> npt.NDArray[np.float64]:
        """
        Function to calculate position at time t
        t: Array of times[sec], shape (N,)
        """

        return self.a0 + self.a1 * t + self.a2 * t ** 2 + self.a3 * t ** 3 + self.a4 * t ** 4

    def velocity(self, t: npt.NDArray[np.float64]) -> npt.NDArray[np.float64]:
        """
        Function to calculate first derivative at time t
        t: Array of times[sec], shape (N,)
        """

        return self.a1 + 2 * self.a2 * t + 3 * self.a3 * t ** 2 + 4 * self.a4 * t ** 3

    def acceleration(self, t: npt.NDArray[np.float64]) -> npt.NDArray[np.float64]:
        """
        Function to calculate second derivative at time t
        t: Array of times[sec], shape (N,)
        """

        return 2 * self.a2 + 6 * self.a3 * t + 12 * self.a4 * t ** 2

    def jerk(self, t: npt.NDArray[np.float64]) -> npt.NDArray[np.float64]:
        """
        Function to calculate third derivative at time t
        t: Array of times[sec], shape (N,)
        """

        return 6 * self.a3 + 24 * self.a4 * t
