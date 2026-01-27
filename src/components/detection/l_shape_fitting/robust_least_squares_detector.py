"""
l_shape_fitting_detector.py

Author: Shisato Yano
"""

import sys
import copy
import itertools
import numpy as np
from collections import deque
from pathlib import Path
from math import pi, sin, cos
from scipy.optimize import least_squares
from math import sin, cos, atan2

abs_dir_path = str(Path(__file__).absolute().parent)
sys.path.append(abs_dir_path + "/../../search/kd_tree")

from l_shape_fitting_detector import LShapeFittingDetector
from kd_tree import KdTree
from rectangle import Rectangle
import numpy as np
from scipy.optimize import least_squares
from math import sin, cos, atan2

class OptimizedLShapeDetector(LShapeFittingDetector):
    def __init__(self, min_rng_th_m=3.0, rng_th_rate=0.1, change_angle_deg=1.0):
        super().__init__(min_rng_th_m, rng_th_rate, change_angle_deg)
        self.f_scale = 0.05  # Tight tolerance for the final polish

    def _calculate_residuals(self, params, points, weights):
        """Standard SDF residuals used for the final polish."""
        cx, cy, theta, w, l = params
        cos_t, sin_t = np.cos(theta), np.sin(theta)
        
        # Transform points to box-local frame
        dx = cos_t * (points[0,:] - cx) + sin_t * (points[1,:] - cy)
        dy = -sin_t * (points[0,:] - cx) + cos_t * (points[1,:] - cy)
        
        # Distance to box edges
        err_x = np.abs(dx) - (w / 2.0)
        err_y = np.abs(dy) - (l / 2.0)
        
        # Signed Distance Field (SDF) logic
        res = np.sqrt(np.maximum(err_x, 0)**2 + np.maximum(err_y, 0)**2) + \
              np.minimum(np.maximum(err_x, err_y), 0)
        
        return res * np.sqrt(weights)

    def _calculate_rectangle(self, points_array):
        # --- STAGE 1: THE ORIGINAL ZHANG SWEEP ---
        # This finds the correct global orientation (0 or 90 deg)
        min_cost_angle = (-float("inf"), None)
        initial_angle_rad = 0.0
        end_angle_rad = np.pi / 2.0 - self.CHANGE_ANGLE_RAD
        
        for angle_rad in np.arange(initial_angle_rad, end_angle_rad, self.CHANGE_ANGLE_RAD):
            rotated_points = self._rotate_points(points_array, angle_rad)
            cost = self._calculate_variance_criterion(rotated_points)
            if min_cost_angle[0] < cost: 
                min_cost_angle = (cost, angle_rad)
        
        # Zhang's result
        best_angle = min_cost_angle[1]
        pts_at_best = self._rotate_points(points_array, best_angle)
        c1, c2 = pts_at_best[0, :], pts_at_best[1, :]
        
        # Initial box parameters from Zhang sweep
        w_init = max(c1) - min(c1)
        l_init = max(c2) - min(c2)
        cx_init = (max(c1) + min(c1)) / 2.0
        cy_init = (max(c2) + min(c2)) / 2.0
        
        # Convert local center back to world coordinates
        cos_a, sin_a = cos(best_angle), sin(best_angle)
        cx_world = cos_a * cx_init - sin_a * cy_init
        cy_world = sin_a * cx_init + cos_a * cy_init
        
        init_guess = [cx_world, cy_world, best_angle, w_init, l_init]

        # --- STAGE 2: THE OPTIMIZATION POLISH ---
        # We constrain the optimizer to only search +/- 5 degrees of the Zhang result.
        # This prevents the 15-degree drift or 45-degree diagonal issues.
        lower = np.array([-np.inf, -np.inf, best_angle - 0.2, w_init*0.8, l_init*0.8])
        upper = np.array([np.inf, np.inf, best_angle + 0.2, w_init*1.2, l_init*1.2])
        
        # Simple uniform weights
        weights = np.ones(points_array.shape[1])
        
        try:
            #print(f"Original params: cx={cx_world:.2f}, cy={cy_world:.2f}, theta={np.degrees(best_angle):.2f} deg, w={w_init:.2f}, l={l_init:.2f}")
            res = least_squares(
                self._calculate_residuals, init_guess,
                args=(points_array, weights),
                bounds=(lower, upper),
                loss='huber', f_scale=.01
            )
            cx, cy, theta, w, l = res.x
            #print(f"Optimization succeeded with cost {res.cost:.4f}")
            #print(f"Refined params: cx={cx:.2f}, cy={cy:.2f}, theta={np.degrees(theta):.2f} deg, w={w:.2f}, l={l:.2f}")
        except:
            # Fallback to Zhang result if optimization fails
            cx, cy, theta, w, l = init_guess

        # --- STAGE 3: CREATE FINAL RECTANGLE ---
        cos_t, sin_t = cos(theta), sin(theta)
        c1_mid = cx * cos_t + cy * sin_t
        c2_mid = -cx * sin_t + cy * cos_t
        
        return Rectangle(a=[cos_t, -sin_t, cos_t, -sin_t],
                         b=[sin_t, cos_t, sin_t, cos_t],
                         c=[c1_mid - w/2.0, c2_mid - l/2.0, c1_mid + w/2.0, c2_mid + l/2.0])
    
    def _apply_temporal_filter(self, current_params, prev_params):
        """
        Smooths the box movement to stop high-frequency jitter.
        """
        # Alpha: 1.0 = No smoothing, 0.1 = Very heavy smoothing
        alpha_pos = 0.4 
        alpha_rot = 0.2 # Lower alpha for rotation makes the heading more stable
        
        smoothed = np.zeros(5)
        # Smooth Position (cx, cy) and Size (w, l)
        for i in [0, 1, 3, 4]:
            smoothed[i] = prev_params[i] + alpha_pos * (current_params[i] - prev_params[i])
            
        # Smooth Rotation (theta) with Angle Wrapping
        diff = (current_params[2] - prev_params[2] + np.pi) % (2 * np.pi) - np.pi
        smoothed[2] = prev_params[2] + alpha_rot * diff
        
        return smoothed
    

class DualLShapeDetector:
    def __init__(self):
        # We initialize both the original and the optimized versions
        self.geometric_detector = LShapeFittingDetector()
        self.optimized_detector = OptimizedLShapeDetector()
        self.latest_rectangles_list = []

    def update(self, point_cloud):
        # 1. Update both detectors
        self.geometric_detector.update(point_cloud)
        self.optimized_detector.update(point_cloud)

        # 2. Combine results for the vehicle state if needed
        # We show both in the list so they can both be drawn
        self.latest_rectangles_list = (self.geometric_detector.latest_rectangles_list + 
                                       self.optimized_detector.latest_rectangles_list)

    def draw(self, axes, elems, x_m, y_m, yaw_rad):
        def custom_draw(rect_obj, color_code):
            """A temporary replacement for the hard-coded draw logic"""
            # Transform contour
            transformed_contour = rect_obj.contour.homogeneous_transformation(x_m, y_m, yaw_rad)
            rectangle_plot, = axes.plot(transformed_contour.get_x_data(),
                                        transformed_contour.get_y_data(),
                                        color=color_code, ls='-', linewidth=2)
            elems.append(rectangle_plot)

            # Transform center
            transformed_center = rect_obj.center_xy.homogeneous_transformation(x_m, y_m, yaw_rad)
            center_plot, = axes.plot(transformed_center.get_x_data(),
                                     transformed_center.get_y_data(),
                                     marker='.', color=color_code)
            elems.append(center_plot)

        # 1. Draw Geometric results in Blue
        for rect in self.geometric_detector.latest_rectangles_list:
            custom_draw(rect, 'b')

        # 2. Draw Optimized results in Red
        for rect in self.optimized_detector.latest_rectangles_list:
            custom_draw(rect, 'r')