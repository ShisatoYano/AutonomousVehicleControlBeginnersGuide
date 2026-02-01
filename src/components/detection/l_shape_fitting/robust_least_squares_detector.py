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
from scipy.spatial.distance import pdist, squareform
class FittingDataTracker:
    def __init__(self):
        # Explicitly initialize the dictionary
        self.gt_cx = 20.0      # replace with 
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
        #ax1.axhline(y=self.gt_cx, color='g', linestyle='-', linewidth=2, label='Ground Truth CX')
        ax1.set_ylabel("X Position [m]")
        ax1.set_title("Center X: Ground Truth vs. Predictions")
        ax1.legend(); ax1.grid(True)

        # Plot 2: CY Accuracy
        ax2.plot(self.data["time"], self.data["cy_geo"], 'b--', label='Zhang CY', alpha=0.5)
        ax2.plot(self.data["time"], self.data["cy_opt"], 'r-', label='Optimized CY')
        #ax2.axhline(y=self.gt_cy, color='g', linestyle='-', linewidth=2, label='Ground Truth CY')
        ax2.set_ylabel("Y Position [m]")
        ax2.set_xlabel("Frames")
        ax2.set_title("Center Y: Ground Truth vs. Predictions")
        ax2.legend(); ax2.grid(True)

        plt.tight_layout()
        plt.show()

# Create the specific instance
tracker = FittingDataTracker()

class OptimizedLShapeDetector(LShapeFittingDetector):
    def __init__(self, min_rng_th_m=3.0, rng_th_rate=0.1, change_angle_deg=1.0, lossfunc='huber', show_logger_debug=True):
        super().__init__(min_rng_th_m, rng_th_rate, change_angle_deg)
        self.f_scale = 0.05  # Tight tolerance for the final polish
        self.prev_rects = {}
        self.lossfunc = lossfunc # Can be Cauchy, Huber, soft_l1, linear
        self.show_logger_debug = show_logger_debug

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

        return np.concatenate([res * np.sqrt(weights)])

    def _get_initial_guess(self, points_array):
        # points_array shape is (2, N)
        points_t = points_array.T # (N, 2)
        
        # 1. Find Best Angle via Variance Criterion -- Global Heading Search
        min_cost_angle = (-float("inf"), None)
        for angle_rad in np.arange(0, np.pi/2 - self.CHANGE_ANGLE_RAD, self.CHANGE_ANGLE_RAD):
            rotated_points = self._rotate_points(points_array, angle_rad)
            cost = self._calculate_variance_criterion(rotated_points)
            if min_cost_angle[0] < cost: 
                min_cost_angle = (cost, angle_rad)
        
        theta_init = min_cost_angle[1]

        # 2. Find the two farthest points (the diameter of the point cloud)
        # and estimate center (cx, cy)
        
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
        
        # 3. Box Dimensions (w, l) Estimation
        cos_t, sin_t = np.cos(theta_init), np.sin(theta_init)
        
        # Local coordinates relative to the chosen center and angle
        dx =  cos_t * (points_array[0,:] - cx) + sin_t * (points_array[1,:] - cy)
        dy = -sin_t * (points_array[0,:] - cx) + cos_t * (points_array[1,:] - cy)
        
        # Now w and l are guaranteed to be aligned with the best angle
        w = np.max(dx) - np.min(dx)
        l = np.max(dy) - np.min(dy)
        
        return [cx, cy, theta_init, max(0.5, w), max(0.5, l)]  # Ensure minimum size to avoid degenerate boxes
    
    
    def _calculate_rectangle(self, points_array):
        self.prev_rects = {}
        
        # --- STAGE 1: Get Initial Guess from L-Shape Fitting (Your Farthest-Point Logic) ---
        init_guess = self._get_initial_guess(points_array)
        
        # --- STAGE 2: Domain for Optimization ---
        theta_window = np.radians(2.0) # Small window is fine now because theta is good
        lower = np.array([-np.inf, -np.inf, init_guess[2] - theta_window, init_guess[3]*0.9, init_guess[4]*0.9])
        upper = np.array([np.inf, np.inf, init_guess[2] + theta_window, init_guess[3]*1.1, init_guess[4]*1.1])
        
        # Simple uniform weights
        # TODO: Implement a New method for weights based on point distribution 
        # This is because points closer to Lidar will have less noise and more denser distribution
        weights = np.ones(points_array.shape[1])

        # Calculate initial cost
        res_before = self._calculate_residuals(init_guess, points_array, weights)
        cost_before = 0.5 * np.sum(res_before**2) # Least squares cost is 0.5 * sum(r^2)
        
        try:
            res = least_squares(
                self._calculate_residuals, init_guess,
                args=(points_array, weights),
                bounds=(lower, upper),
                loss=self.lossfunc, f_scale=.02
            )
            cx, cy, theta, w, l = res.x
            self.prev_rects = res.x

        except:
            # Fallback to Initial Guess if optimization fails
            cx, cy, theta, w, l = init_guess
            self.prev_rects = init_guess
 
        # --- STAGE 3: After Optimization: Logging for Analysis ---
        if 'res' in locals():
            # Calculate residuals after optimization
            
            final_params = res.x
            res_after = self._calculate_residuals(final_params, points_array, weights)
            cost_after = 0.5 * np.sum(res_after**2)
            improvement = (cost_before - cost_after) / (cost_before + 1e-6) * 100
            theta_diff = np.rad2deg(final_params[2] - init_guess[2])
            tracker.log(len(tracker.data["time"]), res_before, res_after, init_guess, res.x)

            # --- PRINT REPORT ---
            if self.show_logger_debug:
                print("-" * 30)
                print(f"Optimization Report (Status: {res.message})")
                print(f"  Cost:  {cost_before:.4f} -> {cost_after:.4f} ({improvement:.1f}% improvement)")
                print(f"  Theta: {np.rad2deg(init_guess[2]):.2f}° -> {np.rad2deg(final_params[2]):.2f}° (Δ: {theta_diff:.2f}°)")
                print("-" * 30)

        else:
            print("Optimization failed; using Initial Guess.")

        # --- STAGE 4: Create Final Rectangle ---
        cos_t, sin_t = cos(theta), sin(theta)
        c1_mid = cx * cos_t + cy * sin_t
        c2_mid = -cx * sin_t + cy * cos_t

        return Rectangle(a=[cos_t, -sin_t, cos_t, -sin_t],
                         b=[sin_t, cos_t, sin_t, cos_t],
                         c=[c1_mid - w/2.0, c2_mid - l/2.0, c1_mid + w/2.0, c2_mid + l/2.0])
    

class DualLShapeDetector:
    def __init__(self, show_logger_debug=True):
        # We initialize both the original and the optimized versions
        self.geometric_detector = LShapeFittingDetector()
        self.optimized_detector = OptimizedLShapeDetector(show_logger_debug=show_logger_debug)
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