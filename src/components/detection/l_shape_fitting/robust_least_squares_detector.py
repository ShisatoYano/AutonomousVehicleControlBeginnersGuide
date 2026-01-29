"""
l_shape_fitting_detector.py

Author: Shisato Yano
"""

from asyncio.log import logger
import sys
import copy
import itertools
import numpy as np
from collections import deque
from pathlib import Path
from math import pi, sin, cos
from scipy.optimize import least_squares
from math import sin, cos, atan2
import matplotlib.pyplot as plt
abs_dir_path = str(Path(__file__).absolute().parent)
sys.path.append(abs_dir_path + "/../../search/kd_tree")

from l_shape_fitting_detector import LShapeFittingDetector
from kd_tree import KdTree
from rectangle import Rectangle
import numpy as np
from scipy.optimize import least_squares
from math import sin, cos, atan2

class FittingDataTracker:
    def __init__(self):
        # Explicitly initialize the dictionary
        self.gt_cx = 20.0      # Example ground truth center x
        self.gt_cy = 0.0   # Example ground truth center y
        self.data = {
            "time": [], 
            "rmse_geo": [], 
            "rmse_opt": [], 
            "theta_geo": [], 
            "theta_opt": [],
            "cx_geo": [], 
            "cy_geo": [], 
            "cx_opt": [], 
            "cy_opt": []
        }

    def log(self, t, r_geo, r_opt, p_geo, p_opt):
        # Append data points to the lists
        self.data["time"].append(t)
        self.data["rmse_geo"].append(np.sqrt(np.mean(r_geo**2)))
        self.data["rmse_opt"].append(np.sqrt(np.mean(r_opt**2)))
        self.data["theta_geo"].append(np.degrees(p_geo[2]))
        self.data["theta_opt"].append(np.degrees(p_opt[2]))
        self.data["cx_geo"].append(p_geo[0])
        self.data["cy_geo"].append(p_geo[1])
        self.data["cx_opt"].append(p_opt[0])
        self.data["cy_opt"].append(p_opt[1])

    def plot(self):
        # Check if we actually collected anything to avoid empty plot errors
        if not self.data["time"]:
            print("No data collected to plot.")
            return

        plt.figure(figsize=(10, 5))
        plt.plot(self.data["time"], self.data["rmse_geo"], 'b--', label='Previous RMSE')
        plt.plot(self.data["time"], self.data["rmse_opt"], 'r-', label='Optimized RMSE')
        plt.ylabel("RMSE [m]"); plt.legend(); plt.grid(True)

        # Create a 2x1 plot specifically for Center Accuracy
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 10))
        
        # Plot 1: CX Accuracy
        ax1.plot(self.data["time"], self.data["cx_geo"], 'b--', label='Zhang CX', alpha=0.5)
        ax1.plot(self.data["time"], self.data["cx_opt"], 'r-', label='Optimized CX')
        ax1.axhline(y=self.gt_cx, color='g', linestyle='-', linewidth=2, label='Ground Truth CX')
        ax1.set_ylabel("X Position [m]")
        ax1.set_title("Center X: Ground Truth vs. Predictions")
        ax1.legend(); ax1.grid(True)

        # Plot 2: CY Accuracy
        ax2.plot(self.data["time"], self.data["cy_geo"], 'b--', label='Zhang CY', alpha=0.5)
        ax2.plot(self.data["time"], self.data["cy_opt"], 'r-', label='Optimized CY')
        ax2.axhline(y=self.gt_cy, color='g', linestyle='-', linewidth=2, label='Ground Truth CY')
        ax2.set_ylabel("Y Position [m]")
        ax2.set_xlabel("Frames")
        ax2.set_title("Center Y: Ground Truth vs. Predictions")
        ax2.legend(); ax2.grid(True)

        plt.tight_layout()
        plt.show()

# Create the specific instance
tracker = FittingDataTracker()

class OptimizedLShapeDetector(LShapeFittingDetector):
    def __init__(self, min_rng_th_m=3.0, rng_th_rate=0.1, change_angle_deg=1.0, lossfunc='huber'):
        super().__init__(min_rng_th_m, rng_th_rate, change_angle_deg)
        self.f_scale = 0.05  # Tight tolerance for the final polish
        self.prev_rects = {}
        self.lossfunc = lossfunc

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
        
        # 3. THE HEADING UNLOCK: Variance Penalty
        # If the box is tilted 15 degrees, the 'dy' values of points on 
        # the 'w' face will have high variance.
        # We penalize the 'thickness' of the walls to force rotation.
        is_w_face = np.abs(dx/w) > np.abs(dy/l)
        is_l_face = ~is_w_face
        
        alignment_penalty = []
        if np.any(is_w_face):
            # Points on the width face should have dy = 0 in a perfect fit
            alignment_penalty.append(np.std(dy[is_w_face]) * 10.0)
        if np.any(is_l_face):
            # Points on the length face should have dx = 0 in a perfect fit
            alignment_penalty.append(np.std(dx[is_l_face]) * 10.0)

        return np.concatenate([res * np.sqrt(weights)])

    def _get_initial_guess(self, points_array):
        # points_array shape is (2, N)
        points_t = points_array.T # (N, 2)
        
        # 1. Find the two farthest points (the diameter of the point cloud)
        # For performance with many points, you could use a convex hull, 
        # but a simple pairwise distance or a search for min/max works for clusters.
        from scipy.spatial.distance import pdist, squareform
        
        if points_t.shape[0] > 2:
            # Finding the absolute farthest pair
            dists = pdist(points_t)
            dist_matrix = squareform(dists)
            i, j = np.unravel_index(np.argmax(dist_matrix), dist_matrix.shape)
            p1, p2 = points_t[i], points_t[j]
            
            # Use the midpoint of these two as the center guess
            cx, cy = (p1 + p2) / 2.0
        else:
            cx, cy = np.mean(points_array, axis=1)
        
        # 1. Temporal Warm-Start
        best_params = self._get_matching_prev_params(points_array)
        #if best_params is not None:
            #return [cx, cy, best_params[2], best_params[3], best_params[4]]
        
        # 2. PCA Fallback
        cov = np.cov(points_array) + np.eye(2) * 1e-6
        evals, evecs = np.linalg.eig(cov)
        sort_indices = np.argsort(evals)[::-1]
        primary_vec = evecs[:, sort_indices[0]]
        theta_init = atan2(primary_vec[1], primary_vec[0])
        
        cos_t, sin_t = cos(theta_init), sin(theta_init)
        dx_raw = cos_t * (points_array[0,:] - cx) + sin_t * (points_array[1,:] - cy)
        dy_raw = -sin_t * (points_array[0,:] - cx) + cos_t * (points_array[1,:] - cy)
        
        w_init = (np.max(dx_raw) - np.min(dx_raw)) + 0.2
        l_init = (np.max(dy_raw) - np.min(dy_raw)) + 0.2
        
        
        return [cx, cy, theta_init, max(0.5, w_init), max(0.5, l_init)]
    
    def _get_matching_prev_params(self, current_points):
        if not self.prev_rects: return None
        cx_curr, cy_curr = np.mean(current_points, axis=1)
        best_match, min_dist = None, 3.0 
        for params in self.prev_rects.values():
            dist = np.hypot(cx_curr - params[0], cy_curr - params[1])
            if dist < min_dist:
                min_dist = dist
                best_match = params
        return best_match
    
    def _calculate_rectangle(self, points_array):
        self.prev_rects = {}
        # --- STAGE 1: THE ORIGINAL ZHANG SWEEP ---
        # This finds the correct global orientation (0 or 90 deg)
        # min_cost_angle = (-float("inf"), None)
        # initial_angle_rad = 0.0
        # end_angle_rad = np.pi / 2.0 - self.CHANGE_ANGLE_RAD
        
        # for angle_rad in np.arange(initial_angle_rad, end_angle_rad, self.CHANGE_ANGLE_RAD):
        #     rotated_points = self._rotate_points(points_array, angle_rad)
        #     cost = self._calculate_variance_criterion(rotated_points)
        #     if min_cost_angle[0] < cost: 
        #         min_cost_angle = (cost, angle_rad)
        
        # # Zhang's result
        # best_angle = min_cost_angle[1]
        # pts_at_best = self._rotate_points(points_array, best_angle)
        # c1, c2 = pts_at_best[0, :], pts_at_best[1, :]
        
        # # Initial box parameters from Zhang sweep
        # w_init = max(c1) - min(c1)
        # l_init = max(c2) - min(c2)
        # cx_init = (max(c1) + min(c1)) / 2.0
        # cy_init = (max(c2) + min(c2)) / 2.0
        
        # # Convert local center back to world coordinates
        # cos_a, sin_a = cos(best_angle), sin(best_angle)
        # cx_world = cos_a * cx_init - sin_a * cy_init
        # cy_world = sin_a * cx_init + cos_a * cy_init
        
        # init_guess = [cx_world, cy_world, best_angle, w_init, l_init]
        # init_guess = self._get_initial_guess(points_array)  # Use improved initial guess
        # # --- STAGE 2: THE OPTIMIZATION POLISH ---
        # # We constrain the optimizer to only search +/- 5 degrees of the Zhang result.
        # # This prevents the 15-degree drift or 45-degree diagonal issues.
        # theta_search_window = np.radians(40.0) # Allow 5 degrees of movement
        
        # --- STAGE 1: ZHANG SWEEP (Global Heading Search) ---
        min_cost_angle = (-float("inf"), None)
        # ... (Your existing 90-degree sweep) ...
        for angle_rad in np.arange(0, np.pi/2 - self.CHANGE_ANGLE_RAD, self.CHANGE_ANGLE_RAD):
            rotated_points = self._rotate_points(points_array, angle_rad)
            cost = self._calculate_variance_criterion(rotated_points)
            if min_cost_angle[0] < cost: 
                min_cost_angle = (cost, angle_rad)
        
        best_angle = min_cost_angle[1]

        # --- STAGE 2: GEOMETRIC CENTERING (Your Farthest-Point Logic) ---
        # Get the good cx, cy from your improved PCA/Farthest-point method
        pca_guess = self._get_initial_guess(points_array)
        
        # 3. FIX: Re-project points to find W and L for THIS specific theta
        # This prevents the 90-degree swap
        cos_t, sin_t = np.cos(best_angle), np.sin(best_angle)
        
        # Local coordinates relative to the chosen center and angle
        dx =  cos_t * (points_array[0,:] - pca_guess[0]) + sin_t * (points_array[1,:] - pca_guess[1])
        dy = -sin_t * (points_array[0,:] - pca_guess[0]) + cos_t * (points_array[1,:] - pca_guess[1])
        
        # Now w and l are guaranteed to be aligned with the Zhang angle
        w_aligned = np.max(dx) - np.min(dx)
        l_aligned = np.max(dy) - np.min(dy)
        # --- STAGE 3: FINAL OPTIMIZATION POLISH ---
        # BLEND THEM: 
        # Take CX, CY from PCA, but THETA from Zhang
        init_guess = [
            pca_guess[0],  # Good cx
            pca_guess[1],  # Good cy
            best_angle,    # Correct global theta
            w_aligned,  # w
            l_aligned   # l
        ]
        theta_window = np.radians(2.0) # Small window is fine now because theta is good
        lower = np.array([-np.inf, -np.inf, best_angle - theta_window, w_aligned*0.9, l_aligned*0.9])
        upper = np.array([np.inf, np.inf, best_angle + theta_window, w_aligned*1.1, l_aligned*1.1])
        
        # Simple uniform weights
        #weights = np.ones(points_array.shape[1])
        weights = self._calculate_smart_weights(points_array)

        # Calculate initial cost
        res_before = self._calculate_residuals(init_guess, points_array, weights)
        cost_before = 0.5 * np.sum(res_before**2) # Least squares cost is 0.5 * sum(r^2)
        
        try:
            #print(f"Original params: cx={cx_world:.2f}, cy={cy_world:.2f}, theta={np.degrees(best_angle):.2f} deg, w={w_init:.2f}, l={l_init:.2f}")
            res = least_squares(
                self._calculate_residuals, init_guess,
                args=(points_array, weights),
                bounds=(lower, upper),
                loss=self.lossfunc, f_scale=.02
                # Explicitly define step sizes
            )
            cx, cy, theta, w, l = res.x
            self.prev_rects = res.x
            #print(f"Optimization succeeded with cost {res.cost:.4f}")
            #print(f"Refined params: cx={cx:.2f}, cy={cy:.2f}, theta={np.degrees(theta):.2f} deg, w={w:.2f}, l={l:.2f}")
        except:
            # Fallback to Zhang result if optimization fails
            cx, cy, theta, w, l = init_guess
            self.prev_rects = init_guess

        # --- STAGE 3: CREATE FINAL RECTANGLE ---
        cos_t, sin_t = cos(theta), sin(theta)
        c1_mid = cx * cos_t + cy * sin_t
        c2_mid = -cx * sin_t + cy * cos_t
        
        # --- AFTER OPTIMIZATION ---
        if 'res' in locals():
            # Send to the global logger
            
            final_params = res.x
            res_after = self._calculate_residuals(final_params, points_array, weights)
            cost_after = 0.5 * np.sum(res_after**2)
            improvement = (cost_before - cost_after) / (cost_before + 1e-6) * 100
            theta_diff = np.rad2deg(final_params[2] - init_guess[2])
            tracker.log(len(tracker.data["time"]), res_before, res_after, init_guess, res.x)

            print("-" * 30)
            print(f"Optimization Report (Status: {res.message})")
            print(f"  Cost:  {cost_before:.4f} -> {cost_after:.4f} ({improvement:.1f}% improvement)")
            print(f"  Theta: {np.rad2deg(init_guess[2]):.2f}° -> {np.rad2deg(final_params[2]):.2f}° (Δ: {theta_diff:.2f}°)")
            print("-" * 30)

        else:
            print("Optimization failed; using Zhang's result.")

        # --- DEBUG REPORT ---
    
        return Rectangle(a=[cos_t, -sin_t, cos_t, -sin_t],
                         b=[sin_t, cos_t, sin_t, cos_t],
                         c=[c1_mid - w/2.0, c2_mid - l/2.0, c1_mid + w/2.0, c2_mid + l/2.0])

    def _calculate_smart_weights(self, points_array):
        # points_array is (2, N)
        # 1. Distance-based weighting (Radial distance from sensor)
        # Assumes sensor is at (0,0) in the local frame or use world coords
        dists = np.linalg.norm(points_array, axis=0)
        # Inverse square law: points twice as far are 4x less certain
        dist_weights = 1.0 / (1.0 + dists**2)

        # 2. Local Density Weighting (Robustness to "Tail Points")
        # We look at how many neighbors each point has within 20cm
        from scipy.spatial import cKDTree
        tree = cKDTree(points_array.T)
        # Find number of neighbors within a 0.3m radius
        neighbor_counts = tree.query_ball_point(points_array.T, r=0.3, return_length=True)
        
        # Normalize density: a point with 1 neighbor is "noisy", 10 is "dense"
        density_weights = np.clip(neighbor_counts / 10.0, 0.1, 1.0)

        # Combined Weights
        final_weights = dist_weights * density_weights
        
        # Normalize so the mean weight is 1.0 (keeps optimizer scaling stable)
        return final_weights / np.mean(final_weights)
    
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
        def custom_draw(rect_obj, color_code, label_prefix, lw=2):
            transformed_contour = rect_obj.contour.homogeneous_transformation(x_m, y_m, yaw_rad)
            rect_plot, = axes.plot(transformed_contour.get_x_data(),
                                   transformed_contour.get_y_data(),
                                   color=color_code, ls='-', linewidth=lw, label=label_prefix)
            elems.append(rect_plot)
            
            transformed_center = rect_obj.center_xy.homogeneous_transformation(x_m, y_m, yaw_rad)
            center_plot, = axes.plot(transformed_center.get_x_data(),
                                     transformed_center.get_y_data(),
                                     marker='.', color=color_code)
            elems.append(center_plot)

        # Draw Geometric (Blue) - Thin line
        for rect in self.geometric_detector.latest_rectangles_list:
            custom_draw(rect, 'b', 'Zhang (Geometric)', lw=1.5)

        # Draw Optimized (Red) - Thick line
        for rect in self.optimized_detector.latest_rectangles_list:
            custom_draw(rect, 'r', 'Optimized (Polish)', lw=3.0)