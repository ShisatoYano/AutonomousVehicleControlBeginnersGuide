"""
Path Planning sample program by
Rapidly-Exploring Random Trees.

Author: Shisato Yano
"""

import matplotlib.pyplot as plt
import numpy as np
import math

# parameters
START_X_M = 10.0 # start point
START_Y_M = 10.0
GOAL_X_M = 50.0 # goal point
GOAL_Y_M = 50.0
MIN_SAMPLING_AREA_M = 0 # random sampling area
MAX_SAMPLING_AREA_M = 60
OBSTACLE_RADIUS_M = 0.5
VEHICLE_SIZE_HALF_M = 2.0

# flag to switch showing output graph or not
show_plot = True


class ObstacleMap:
    def __init__(self, a_axes, a_start_x_m, a_start_y_m, a_goal_x_m, 
                 a_goal_y_m, a_obst_radius_m, a_vehicle_size_half_m):
        # set parameters
        self.o_start_x_m = a_start_x_m
        self.o_start_y_m = a_start_y_m
        self.o_goal_x_m = a_goal_x_m
        self.o_goal_y_m = a_goal_y_m
        self.o_obst_radius_m = a_obst_radius_m
        self.o_vehicle_size_half_m = a_vehicle_size_half_m

        self.set_obstacles_position_size()
        self.calculate_map_range()

        if show_plot:
            for obst_x, obst_y, obst_size in zip(self.o_obst_x_list, self.o_obst_y_list, self.o_obst_size_list):
                self.plot_circle(a_axes, obst_x, obst_y, obst_size)
            a_axes.plot(self.o_start_x_m, self.o_start_y_m, "og")
            a_axes.plot(self.o_goal_x_m, self.o_goal_y_m, "xb")
    
    def set_obstacles_position_size(self):
        self.o_obst_x_list = []
        self.o_obst_y_list = []
        self.o_obst_size_list = []
        for i in range(0, 60): 
            self.o_obst_x_list.append(i)
            self.o_obst_y_list.append(0.0)
            self.o_obst_size_list.append(self.o_obst_radius_m)
        for i in range(0, 60): 
            self.o_obst_x_list.append(60.0) 
            self.o_obst_y_list.append(i)
            self.o_obst_size_list.append(self.o_obst_radius_m)
        for i in range(0, 61): 
            self.o_obst_x_list.append(i)
            self.o_obst_y_list.append(60.0)
            self.o_obst_size_list.append(self.o_obst_radius_m)
        for i in range(0, 60): 
            self.o_obst_x_list.append(0.0)
            self.o_obst_y_list.append(i)
            self.o_obst_size_list.append(self.o_obst_radius_m)
        for i in range(0, 40): 
            self.o_obst_x_list.append(20.0)
            self.o_obst_y_list.append(i)
            self.o_obst_size_list.append(self.o_obst_radius_m)
        for i in range(0, 40): 
            self.o_obst_x_list.append(40.0)
            self.o_obst_y_list.append(60.0-i)
            self.o_obst_size_list.append(self.o_obst_radius_m)
    
    def calculate_map_range(self):
        self.o_min_x_m = round(min(self.o_obst_x_list))
        self.o_min_y_m = round(min(self.o_obst_y_list))
        self.o_max_x_m = round(max(self.o_obst_x_list))
        self.o_max_y_m = round(max(self.o_obst_y_list))
        print("Min X[m]:", self.o_min_x_m)
        print("Min Y[m]:", self.o_min_y_m)
        print("Max X[m]:", self.o_max_x_m)
        print("Max Y[m]:", self.o_max_y_m)
    
    def plot_circle(self, a_axes, a_x, a_y, a_size, a_color="k"):
        angle_deg_list = list(range(0, 360, 5))
        angle_deg_list.append(0)
        x_list = [a_x + a_size * math.cos(np.deg2rad(deg)) for deg in angle_deg_list]
        y_list = [a_y + a_size * math.sin(np.deg2rad(deg)) for deg in angle_deg_list]
        a_axes.fill(x_list, y_list, a_color)


class RRT:
    def __init__(self, a_axes, a_map, a_min_samp, a_max_samp, a_expand_dist_m=3.0,
                 a_path_reso_m=0.5, a_goal_samp_rate=5.0, a_max_iter=500):
        self.o_map = a_map
    
    class Node:
        def __init__(self, a_x_m, a_y_m):
            self.o_x_m = a_x_m
            self.o_y_m = a_y_m
            self.o_x_path = []
            self.o_y_path = []
            self.o_parent_node = None


def main():
    print(__file__ + " start!!")

    # initialize plot
    ax = plt.subplot(1, 1, 1)
    ax.set_xlabel("X[m]")
    ax.set_ylabel("Y[m]")
    ax.set_aspect("equal")
    ax.grid(True)

    # generate map instance
    om = ObstacleMap(ax, START_X_M, START_Y_M, GOAL_X_M, GOAL_Y_M, 
                     OBSTACLE_RADIUS_M, VEHICLE_SIZE_HALF_M)

    # only when show plot flag is true, show output graph
    # when unit test is executed, this flag become false
    # and the graph is not shown
    if show_plot:
        plt.show()

    return True


# execute main process
if __name__ == "__main__":
    main()
