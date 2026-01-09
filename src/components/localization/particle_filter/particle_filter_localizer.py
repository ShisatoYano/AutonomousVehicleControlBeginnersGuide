"""
particle_filter_localizer.py

Author: Sahruday Patti (based on EKF/UKF patterns by Shisato Yano and Bruno DOKPOMIWA)

Particle Filter implementation for vehicle localization.
This is a Monte Carlo based approach that represents the posterior distribution
using a set of weighted particles (samples).
"""

import sys
import numpy as np
from pathlib import Path
from math import cos, sin, sqrt, atan2, pi

sys.path.append(str(Path(__file__).absolute().parent) + "/../../state")
sys.path.append(str(Path(__file__).absolute().parent) + "/../../array")
sys.path.append(str(Path(__file__).absolute().parent) + "/../../sensors/gnss")
from state import State
from xy_array import XYArray


class ParticleFilterLocalizer:
    """
    Self localization by Particle Filter class.
    
    The Particle Filter (also known as Sequential Monte Carlo) represents
    the belief about the robot's state using a set of weighted samples (particles).
    
    Key advantages over Kalman Filters:
    - Can handle arbitrary nonlinear systems
    - Can represent multimodal distributions
    - Can handle non-Gaussian noise
    
    Algorithm steps:
    1. PREDICT: Propagate each particle through the motion model with noise
    2. UPDATE: Weight particles based on observation likelihood
    3. RESAMPLE: Draw new particles based on weights (if needed)
    4. ESTIMATE: Compute weighted mean for state estimate
    """

    # Resampling method constants
    RESAMPLE_MULTINOMIAL = 'multinomial'
    RESAMPLE_SYSTEMATIC = 'systematic'
    RESAMPLE_LOW_VARIANCE = 'low_variance'

    def __init__(self, num_particles=500,
                 accel_noise=0.2, yaw_rate_noise=10.0,
                 obsrv_x_noise=1.0, obsrv_y_noise=1.0,
                 resampling_method='systematic',
                 resample_threshold=0.5,
                 color='r'):
        """
        Constructor
        
        num_particles: Number of particles to use (default 500)
        accel_noise: Standard deviation of acceleration noise [m/s^2]
        yaw_rate_noise: Standard deviation of yaw rate noise [deg/s]
        obsrv_x_noise: Standard deviation of x observation noise [m]
        obsrv_y_noise: Standard deviation of y observation noise [m]
        resampling_method: One of 'multinomial', 'systematic', 'low_variance'
        resample_threshold: Threshold for effective sample size ratio (0-1)
        color: Color for drawing particles and uncertainty ellipse
        """
        
        # Number of particles
        self.num_particles = num_particles
        
        # State dimension (x, y, yaw, speed)
        self.DIM_NUM = 4
        
        # Noise covariance matrices
        self.INPUT_NOISE_VAR_MAT = np.diag([accel_noise, np.deg2rad(yaw_rate_noise)]) ** 2
        self.OBSRV_NOISE_VAR_MAT = np.diag([obsrv_x_noise, obsrv_y_noise]) ** 2
        
        # Observation noise standard deviations for likelihood computation
        self.obsrv_x_std = obsrv_x_noise
        self.obsrv_y_std = obsrv_y_noise
        
        # Resampling configuration
        self.resampling_method = resampling_method
        self.resample_threshold = resample_threshold
        
        # Initialize particles and weights
        # Particles: 4 x num_particles array
        # Each column is a particle [x, y, yaw, speed]^T
        self.particles = np.zeros((self.DIM_NUM, num_particles))
        self.weights = np.ones(num_particles) / num_particles
        
        # Estimated state and covariance (for compatibility with EKF/UKF interface)
        self.state = np.zeros((self.DIM_NUM, 1))
        self.cov_mat = np.eye(self.DIM_NUM)
        
        # Visualization color
        self.DRAW_COLOR = color
        
        # Flag to track if particles have been initialized
        self._initialized = False

    def _initialize_particles(self, state):
        """
        Private function to initialize particles around the initial state.
        
        state: Initial state vector [x, y, yaw, speed]^T
        """
        
        # Initialize particles with small noise around initial state
        init_std = np.array([[1.0], [1.0], [0.1], [0.1]])  # Initial uncertainty
        
        for i in range(self.num_particles):
            noise = init_std * np.random.randn(self.DIM_NUM, 1)
            self.particles[:, i:i+1] = state + noise
        
        self._initialized = True

    def _predict_particles(self, input_vec, time_s):
        """
        Private function to predict (propagate) all particles through the motion model.
        Each particle is moved according to the motion model with added process noise.
        
        input_vec: Input vector [accel, yaw_rate]^T
        time_s: Time interval [sec]
        """
        
        for i in range(self.num_particles):
            # Add process noise to each particle's input
            input_noise = np.sqrt(self.INPUT_NOISE_VAR_MAT) @ np.random.randn(2, 1)
            noisy_input = input_vec + input_noise
            
            # Propagate particle through motion model
            particle = self.particles[:, i:i+1]
            self.particles[:, i:i+1] = State.motion_model(particle, noisy_input, time_s)

    def _compute_weights(self, gnss_observation):
        """
        Private function to compute particle weights based on observation likelihood.
        
        The weight of each particle is proportional to p(z | x_i), 
        the probability of the observation given the particle's state.
        
        gnss_observation: Observed position [[x], [y]] from GNSS
        """
        
        for i in range(self.num_particles):
            # Get particle's predicted observation (x, y position)
            particle_x = self.particles[0, i]
            particle_y = self.particles[1, i]
            
            # Compute observation error
            dx = gnss_observation[0, 0] - particle_x
            dy = gnss_observation[1, 0] - particle_y
            
            # Compute likelihood using Gaussian probability
            # p(z|x) = exp(-0.5 * (dx^2/sigma_x^2 + dy^2/sigma_y^2))
            likelihood = np.exp(-0.5 * ((dx ** 2) / (self.obsrv_x_std ** 2) + 
                                        (dy ** 2) / (self.obsrv_y_std ** 2)))
            
            # Update weight
            self.weights[i] *= likelihood
        
        # Normalize weights
        weight_sum = np.sum(self.weights)
        if weight_sum > 1e-10:
            self.weights /= weight_sum
        else:
            # If all weights are near zero, reset to uniform
            self.weights = np.ones(self.num_particles) / self.num_particles

    def _compute_effective_sample_size(self):
        """
        Private function to compute the effective sample size (ESS).
        
        ESS measures the degeneracy of the particle set.
        ESS = 1 / sum(w_i^2)
        
        When ESS is low (particles have very unequal weights), resampling is needed.
        
        Returns: Effective sample size (1 to num_particles)
        """
        
        return 1.0 / np.sum(self.weights ** 2)

    def _resample_multinomial(self):
        """
        Multinomial resampling.
        
        The simplest resampling strategy: draw N new particles from the current
        set with probabilities proportional to their weights.
        
        Intuition: Particles with higher weights are more likely to be selected.
        """
        
        # Generate indices by sampling from categorical distribution
        indices = np.random.choice(
            self.num_particles,
            size=self.num_particles,
            replace=True,
            p=self.weights
        )
        
        # Resample particles
        self.particles = self.particles[:, indices]
        
        # Reset weights to uniform
        self.weights = np.ones(self.num_particles) / self.num_particles

    def _resample_systematic(self):
        """
        Systematic resampling.
        
        Uses a single random number and evenly spaced points to select particles.
        This ensures more uniform coverage of the weight space.
        
        Intuition: Like spinning a roulette wheel with evenly spaced pointers.
        """
        
        # Compute cumulative sum of weights
        cumsum = np.cumsum(self.weights)
        
        # Generate evenly spaced points with single random offset
        positions = (np.arange(self.num_particles) + np.random.random()) / self.num_particles
        
        # Select indices based on positions
        indices = np.zeros(self.num_particles, dtype=int)
        j = 0
        for i in range(self.num_particles):
            while positions[i] > cumsum[j]:
                j += 1
            indices[i] = j
        
        # Resample particles
        self.particles = self.particles[:, indices]
        
        # Reset weights to uniform
        self.weights = np.ones(self.num_particles) / self.num_particles

    def _resample_low_variance(self):
        """
        Low-variance resampling (also known as stratified resampling).
        
        Similar to systematic but provides better variance properties.
        Each stratum gets exactly one sample.
        
        This is often considered the best resampling strategy as it:
        - Minimizes variance in the number of offspring
        - Prevents particle deprivation
        """
        
        # Compute cumulative sum of weights
        cumsum = np.cumsum(self.weights)
        
        # Single random number for all selections
        r = np.random.random() / self.num_particles
        
        # Select indices
        indices = np.zeros(self.num_particles, dtype=int)
        j = 0
        for i in range(self.num_particles):
            u = r + i / self.num_particles
            while u > cumsum[j]:
                j += 1
            indices[i] = j
        
        # Resample particles
        self.particles = self.particles[:, indices]
        
        # Reset weights to uniform
        self.weights = np.ones(self.num_particles) / self.num_particles

    def _resample(self):
        """
        Private function to resample particles using the selected method.
        Only resamples if effective sample size is below threshold.
        """
        
        # Compute effective sample size
        ess = self._compute_effective_sample_size()
        threshold = self.resample_threshold * self.num_particles
        
        # Only resample if ESS is below threshold
        if ess < threshold:
            if self.resampling_method == self.RESAMPLE_MULTINOMIAL:
                self._resample_multinomial()
            elif self.resampling_method == self.RESAMPLE_SYSTEMATIC:
                self._resample_systematic()
            elif self.resampling_method == self.RESAMPLE_LOW_VARIANCE:
                self._resample_low_variance()
            else:
                # Default to systematic if unknown method
                self._resample_systematic()

    def _estimate_state(self):
        """
        Private function to estimate the state from the weighted particle set.
        
        Computes the weighted mean of all particles.
        Uses circular mean for yaw angle to handle wraparound correctly.
        Also computes the covariance for visualization.
        
        Returns: Estimated state as 4x1 array
        """
    
        # Weighted mean for x, y, speed (linear quantities)
        est_x = np.sum(self.weights * self.particles[0, :])
        est_y = np.sum(self.weights * self.particles[1, :])
        est_speed = np.sum(self.weights * self.particles[3, :])
        
        # Circular mean for yaw angle
        # The arithmetic mean fails when angles wrap around (e.g., mean of 350° and 10° is NOT 180°)
        # Instead, we convert to unit vectors, average those, then convert back to angle
        sin_sum = np.sum(self.weights * np.sin(self.particles[2, :]))
        cos_sum = np.sum(self.weights * np.cos(self.particles[2, :]))
        est_yaw = np.arctan2(sin_sum, cos_sum)
        
        est_state = np.array([[est_x], [est_y], [est_yaw], [est_speed]])
        
        # Weighted covariance
        cov = np.zeros((self.DIM_NUM, self.DIM_NUM))
        for i in range(self.num_particles):
            diff = self.particles[:, i:i+1] - est_state
            
            # Wrap yaw difference to [-pi, pi]
            # Without this, a particle at -170° and mean at 170° would show diff of -340° instead of 20°
            diff[2, 0] = np.arctan2(np.sin(diff[2, 0]), np.cos(diff[2, 0]))
            
            cov += self.weights[i] * (diff @ diff.T)
        
        self.state = est_state
        self.cov_mat = cov
    
        return est_state

    def update(self, state, accel_mps2, yaw_rate_rps, time_s, gnss):
        """
        Main update function following the EKF/UKF interface.
        
        Performs the full Particle Filter cycle:
        1. Initialize particles (if first call)
        2. Predict: propagate particles through motion model
        3. Update: compute weights from observation likelihood
        4. Resample: draw new particles if needed
        5. Estimate: compute weighted mean
        
        state: Last estimated state data (State object)
        accel_mps2: Acceleration input from controller [m/s^2]
        yaw_rate_rps: Yaw rate input from controller [rad/s]
        time_s: Simulation interval time [sec]
        gnss: GNSS observation data [[x], [y]]
        
        Return: Estimated state as 4x1 array [[x], [y], [yaw], [speed]]
        """
        
        # Get current state as array
        last_state = np.array([[state.get_x_m()],
                               [state.get_y_m()],
                               [state.get_yaw_rad()],
                               [state.get_speed_mps()]])
        
        # Initialize particles around initial state if not done
        if not self._initialized:
            self._initialize_particles(last_state)
        
        # Create input vector
        input_vec = np.array([[accel_mps2],
                              [yaw_rate_rps]])
        
        # Step 1: PREDICT - propagate particles through motion model
        self._predict_particles(input_vec, time_s)
        
        # Step 2: UPDATE - compute weights from observation likelihood
        self._compute_weights(gnss)
        
        # Step 3: RESAMPLE - draw new particles if effective sample size is low
        self._resample()
        
        # Step 4: ESTIMATE - compute weighted mean for state estimate
        est_state = self._estimate_state()
        
        return est_state

    def draw(self, axes, elems, pose):
        """
        Function to draw particles, estimated position, and uncertainty ellipse.
        
        axes: Axes object of figure
        elems: List of plot objects
        pose: Vehicle's pose [[x], [y], [yaw]]
        """
        
        # Draw particles as scatter points
        # Alpha (transparency) is proportional to weight for visibility
        max_weight = np.max(self.weights) if np.max(self.weights) > 0 else 1.0
        normalized_weights = self.weights / max_weight
        
        # Draw particles with varying alpha based on weight
        particle_plot = axes.scatter(
            self.particles[0, :],
            self.particles[1, :],
            c=self.DRAW_COLOR,
            s=5,  # Small marker size
            alpha=0.3 + 0.7 * normalized_weights,  # Alpha varies from 0.3 to 1.0
            marker='.'
        )
        elems.append(particle_plot)
        
        # Draw estimated position marker
        est_marker, = axes.plot(
            self.state[0, 0],
            self.state[1, 0],
            marker='x',
            markersize=10,
            color=self.DRAW_COLOR,
            markeredgewidth=2
        )
        elems.append(est_marker)
        
        # Draw uncertainty ellipse (same as EKF/UKF)
        # Extract 2x2 covariance for x and y
        xy_cov = self.cov_mat[:2, :2]
        
        # Handle edge case where covariance is too small or negative
        try:
            eig_val, eig_vec = np.linalg.eig(xy_cov)
            eig_val = np.real(eig_val)
            eig_vec = np.real(eig_vec)
            
            # Ensure positive eigenvalues
            eig_val = np.maximum(eig_val, 1e-6)
            
            if eig_val[0] >= eig_val[1]:
                big_idx, small_idx = 0, 1
            else:
                big_idx, small_idx = 1, 0
            
            # 3-sigma ellipse (99.7% confidence)
            a = sqrt(3.0 * eig_val[big_idx])
            b = sqrt(3.0 * eig_val[small_idx])
            angle = atan2(eig_vec[1, big_idx], eig_vec[0, big_idx])
            
            # Generate ellipse points
            t = np.arange(0, 2 * pi + 0.1, 0.1)
            xs = [a * cos(it) for it in t]
            ys = [b * sin(it) for it in t]
            xys = np.array([xs, ys])
            xys_array = XYArray(xys)
            
            # Transform ellipse to estimated position
            transformed_xys = xys_array.homogeneous_transformation(
                self.state[0, 0],
                self.state[1, 0],
                angle
            )
            
            elip_plot, = axes.plot(
                transformed_xys.get_x_data(),
                transformed_xys.get_y_data(),
                color=self.DRAW_COLOR,
                linewidth=1.5
            )
            elems.append(elip_plot)
            
        except np.linalg.LinAlgError:
            # Skip ellipse drawing if eigenvalue computation fails
            pass

    def get_particles(self):
        """
        Function to get the current particle set.
        
        Returns: 4 x num_particles numpy array
        """
        
        return self.particles

    def get_weights(self):
        """
        Function to get the current weights.
        
        Returns: 1D numpy array of length num_particles
        """
        
        return self.weights

    def get_effective_sample_size(self):
        """
        Function to get the current effective sample size.
        
        Returns: Scalar value between 1 and num_particles
        """
        
        return self._compute_effective_sample_size()

