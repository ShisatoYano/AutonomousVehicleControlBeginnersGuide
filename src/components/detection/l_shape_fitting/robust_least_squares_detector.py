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
from math import sin, cos
from scipy.optimize import least_squares
from math import sin, cos, atan2

abs_dir_path = str(Path(__file__).absolute().parent)
sys.path.append(abs_dir_path + "/../../search/kd_tree")

from l_shape_fitting_detector import LShapeFittingDetector
from kd_tree import KdTree
from rectangle import Rectangle
    

class RobustRectangleFittingDetector(LShapeFittingDetector):
    def __init__(self, min_rng_th_m=3.0, rng_th_rate=0.1, f_scale=1.0):
        super().__init__(min_rng_th_m, rng_th_rate)
        self.f_scale = f_scale  # Outlier threshold (m) for robust loss
        # Temporal storage: {object_id: [cx, cy, theta, w, l]}
        self.prev_rects = {}
        self.history = []  # [time, pred_w, pred_l, true_w, true_l]
        self.time_count = 0.0
        self.dt = 0.1 # Simulation step

    def _get_initial_guess(self, points_array):
        cx, cy = np.mean(points_array, axis=1)
        
        # Search for a nearby rectangle from the previous frame
        best_id = None
        min_dist = 3.0 # 2-meter tracking threshold
        
        for rect_id, params in self.prev_rects.items():
            dist = np.hypot(cx - params[0], cy - params[1])
            if dist < min_dist:
                min_dist = dist
                best_id = rect_id
        
        # If found, use previous params as the warm start
        if best_id is not None:
            return self.prev_rects[best_id]
        
        # Enhanced PCA Initializer
        # 2. PCA for Orientation
        cov = np.cov(points_array)
        evals, evecs = np.linalg.eig(cov)
        sort_indices = np.argsort(evals)[::-1]
        primary_vec = evecs[:, sort_indices[0]]
        theta_init = atan2(primary_vec[1], primary_vec[0])
        
        # 3. Transform to Local Frame to find TRUE extents
        cos_t, sin_t = cos(theta_init), sin(theta_init)
        # Calculate local coordinates relative to a temporary origin (the mean)
        mx, my = np.mean(points_array, axis=1)
        dx_raw = cos_t * (points_array[0,:] - mx) + sin_t * (points_array[1,:] - my)
        dy_raw = -sin_t * (points_array[0,:] - mx) + cos_t * (points_array[1,:] - my)
        
        # 4. Calculate dimensions
        w_init = (np.max(dx_raw) - np.min(dx_raw)) + 0.1
        l_init = (np.max(dy_raw) - np.min(dy_raw)) + 0.1
        
        # 5. Calculate NEW center based on the midpoint of extents
        # This is more robust for "long big rectangles" than the mean
        cx_local = (np.max(dx_raw) + np.min(dx_raw)) / 2.0
        cy_local = (np.max(dy_raw) + np.min(dy_raw)) / 2.0
        
        # Rotate local center back to global frame
        cx = mx + (cx_local * cos_t - cy_local * sin_t)
        cy = my + (cx_local * sin_t + cy_local * cos_t)
        
        return [cx, cy, theta_init, w_init, l_init]

    def _calculate_residuals(self, params, points, prev_params=None):
        """
        Calculates distance to closest rectangle edge.
        """
        cx, cy, theta, w, l = params
        cos_t, sin_t = np.cos(theta), np.sin(theta)
        
        dx = cos_t * (points[0,:] - cx) + sin_t * (points[1,:] - cy)
        dy = -sin_t * (points[0,:] - cx) + cos_t * (points[1,:] - cy)
        
        # 1. Edge distances (Core Data)
        res_x = np.abs(np.abs(dx) - w/2.0)
        res_y = np.abs(np.abs(dy) - l/2.0)
        base_res = np.minimum(res_x, res_y)
        
        # 2. Constraints (Using np.maximum to avoid the ValueError)
        span_w = np.max(dx) - np.min(dx)
        span_l = np.max(dy) - np.min(dy)
        
        # Penalty if box is much BIGGER than points (Stops 18m explosion)
        tightness_penalty = np.maximum(0, np.array([w - (span_w + 0.5), l - (span_l + 0.5)])) * 2.0
        
        # Penalty if box is much SMALLER than 0.8m (Stops the 0.2m collapse)
        min_size_penalty = np.maximum(0, np.array([0.8 - w, 0.8 - l])) * 5.0

        # 3. Momentum
        momentum = []
        if prev_params is not None:
            # prev_params is likely a list or array [cx, cy, theta, w, l]
            momentum = [(w - prev_params[3]) * 0.3, (l - prev_params[4]) * 0.3]

        return np.concatenate([base_res, tightness_penalty, min_size_penalty, momentum])

    def _create_rectangle_obj(self, cx, cy, theta, w, l):
        """
        Helper to map [cx, cy, theta, w, l] back to Yano's Rectangle(a, b, c).
        """
        cos_t, sin_t = cos(theta), sin(theta)
        c1_mid = cx * cos_t + cy * sin_t
        c2_mid = -cx * sin_t + cy * cos_t
        
        c_params = [c1_mid - w/2.0, c2_mid - l/2.0, 
                    c1_mid + w/2.0, c2_mid + l/2.0]
        
        return Rectangle(a=[cos_t, -sin_t, cos_t, -sin_t],
                         b=[sin_t, cos_t, sin_t, cos_t],
                         c=c_params)

    def _optimize_rectangle(self, points_array, prev_params=None):
        # 1. Get the initial guess from PCA
        initial_params = self._get_initial_guess(points_array)
        
        # If we have previous params, use them as the warm start (initial guess)
        if prev_params is not None:
            initial_params = prev_params
            # We still update the center to the current mean to keep up with motion
            initial_params[0], initial_params[1] = np.mean(points_array, axis=1)
        # 2. Define bounds (as you have them)
        # [cx, cy, theta, w, l]
        lower_bounds = [-np.inf, -np.inf, -np.pi, 0.2, 0.2]
        upper_bounds = [np.inf, np.inf, np.pi, 40.0, 40.0]

        # 3. SAFETY: Clip the initial guess to be strictly within bounds
        # This prevents the "x0 is infeasible" error
        for i in range(len(initial_params)):
            # We use a small epsilon offset to ensure it's not exactly on the edge
            eps = 1e-3
            initial_params[i] = np.clip(initial_params[i], 
                                        lower_bounds[i] + eps, 
                                        upper_bounds[i] - eps)
        
        # 4. Handle Angle Wrapping specifically
        # Ensure theta is in [-pi, pi]
        #initial_params[2] = atan2(sin(initial_params[2]), cos(initial_params[2]))

        # 5. Run the optimizer
        res = least_squares(
            self._calculate_residuals,
            initial_params,
            args=(points_array,prev_params),
            loss='soft_l1',
            f_scale= .5,
            bounds=(lower_bounds, upper_bounds),
            method='trf'
        )
        return res.x

    def _search_rectangles(self, clusters_list):
        new_rects_params = {}
        rectangles_list = []
        
        print(f"\n--- Frame Debug: {len(clusters_list)} clusters found by LiDAR ---")

        for i, cluster in enumerate(clusters_list):
            # 1. Check Point Count
            if len(cluster) < 5: 
                print(f"  Cluster {i}: Dropped (Too sparse: {len(cluster)} points)")
                continue
                
            array_list = [point.get_point_array() for point in list(cluster)]
            points = np.concatenate(array_list, 1)

            # 2. Check Data Association
            prev_params = self._get_matching_prev_params(points)
            if prev_params:
                print(f"  Cluster {i}: Matched to previous Object (Center: {prev_params[0]:.1f}, {prev_params[1]:.1f})")
            else:
                print(f"  Cluster {i}: No match, starting fresh (PCA)")

            try:
                # 3. Check Optimization
                opt_params = self._optimize_rectangle(points, prev_params)
                
                # Check for "Exploding" dimensions
                if opt_params[3] > 15.0 or opt_params[4] > 15.0:
                    print(f"  Cluster {i}: WARNING! Dimensions exploded (w={opt_params[3]:.1f}, l={opt_params[4]:.1f})")

                new_rects_params[i] = opt_params
                rectangles_list.append(self._create_rectangle_obj(*opt_params))
                print(f"  Cluster {i}: SUCCESS (w={opt_params[3]:.2f}, l={opt_params[4]:.2f})")

            except Exception as e:
                # This catches the "x0 is infeasible" and other math errors
                print(f"  Cluster {i}: FAILED Optimization: {str(e)}")

        self.prev_rects = new_rects_params
        self.latest_rectangles_list = rectangles_list
    
    def _get_matching_prev_params(self, current_points):
        """
        Finds the parameters from the previous frame that 
        belong to the current cluster based on center distance.
        """
        if not self.prev_rects:
            return None

        # Calculate current cluster center (rough estimate)
        cx_curr, cy_curr = np.mean(current_points, axis=1)
        
        best_match_params = None
        min_dist = 2.5 # Threshold in meters (adjustable based on vehicle speed)

        for rect_id, params in self.prev_rects.items():
            # params: [cx, cy, theta, w, l]
            prev_cx, prev_cy = params[0], params[1]
            dist = np.hypot(cx_curr - prev_cx, cy_curr - prev_cy)
            
            if dist < min_dist:
                min_dist = dist
                best_match_params = params
        
        return best_match_params