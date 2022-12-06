"""
Path Planning sample program by
RRT(Rapidly-Exploring Random Trees)*.

Author: Shisato Yano
"""

import matplotlib.pyplot as plt
import numpy as np
import math
import random

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
    
    def is_inside(self, a_x, a_y):
        return (a_x >= self.o_min_x_m) and (a_x <= self.o_max_x_m) \
            and (a_y >= self.o_min_y_m) and (a_y <= self.o_max_y_m)
    
    def is_safe(self, a_x_list, a_y_list):
        for o_x, o_y, o_size in zip(self.o_obst_x_list, self.o_obst_y_list, self.o_obst_size_list):
            diff_x_list = [o_x - x for x in a_x_list]
            diff_y_list = [o_y - y for y in a_y_list]
            diff_list = [d_x * d_x + d_y * d_y for (d_x, d_y) in zip(diff_x_list, diff_y_list)]

            if min(diff_list) <= (o_size + self.o_vehicle_size_half_m)**2:
                return False

        return True
    
    def plot_circle(self, a_axes, a_x, a_y, a_size, a_color="k"):
        angle_deg_list = list(range(0, 360, 5))
        angle_deg_list.append(0)
        x_list = [a_x + a_size * math.cos(np.deg2rad(deg)) for deg in angle_deg_list]
        y_list = [a_y + a_size * math.sin(np.deg2rad(deg)) for deg in angle_deg_list]
        a_axes.fill(x_list, y_list, a_color)


class RRTStar:
    def __init__(self, a_axes, a_map, a_min_samp, a_max_samp, a_expand_th_m=3.0,
                 a_path_reso_m=0.5, a_goal_samp_rate=5.0, a_max_iter=1000,
                 a_connect_dist=50.0):
        # set parameters
        self.o_axes = a_axes
        self.o_map = a_map
        self.o_start = self.Node(self.o_map.o_start_x_m, self.o_map.o_start_y_m)
        self.o_goal = self.Node(self.o_map.o_goal_x_m, self.o_map.o_goal_y_m)
        self.o_min_samp = a_min_samp
        self.o_max_samp = a_max_samp
        self.o_expand_th_m = a_expand_th_m
        self.o_path_reso_m = a_path_reso_m
        self.o_goal_samp_rate = a_goal_samp_rate
        self.o_max_iter = a_max_iter
        self.o_connect_dist = a_connect_dist
        self.o_node_list = []
        self.o_final_path = []

        self.plot_samp_node, = a_axes.plot([], [], "xc")
        self.plot_final_path, = a_axes.plot([], [], "-r")
    
    class Node:
        def __init__(self, a_x_m, a_y_m):
            self.o_x_m = a_x_m
            self.o_y_m = a_y_m
            self.o_cost = 0.0
            self.o_x_path = []
            self.o_y_path = []
            self.o_parent_node = None
    
    def random_sampling_node(self):
        if random.randint(0, 100) > self.o_goal_samp_rate:
            node = self.Node(random.uniform(self.o_min_samp, self.o_max_samp),
                             random.uniform(self.o_min_samp, self.o_max_samp))
        else: # goal bias sampling
            node = self.Node(self.o_goal.o_x_m, self.o_goal.o_y_m)
        return node
    
    def get_nearest_node(self, a_sampled_node):
        dist_list = [(node.o_x_m - a_sampled_node.o_x_m)**2 + (node.o_y_m - a_sampled_node.o_y_m)**2 
                     for node in self.o_node_list]
        nearest_index = dist_list.index(min(dist_list))
        return self.o_node_list[nearest_index]
    
    def calculate_distance_angle(self, a_from, a_to):
        diff_x_m = a_to.o_x_m - a_from.o_x_m
        diff_y_m = a_to.o_y_m - a_from.o_y_m
        dist_m = math.hypot(diff_x_m, diff_y_m)
        angle_rad = math.atan2(diff_y_m, diff_x_m)
        return dist_m, angle_rad
    
    def expand_node(self, a_from, a_to):
        new_node = self.Node(a_from.o_x_m, a_from.o_y_m)

        dist_m, angle_rad = self.calculate_distance_angle(new_node, a_to)

        new_node.o_x_path = [new_node.o_x_m]
        new_node.o_y_path = [new_node.o_y_m]

        expand_dist_m = float("inf")
        if self.o_expand_th_m > dist_m: expand_dist_m  =  dist_m
        else: expand_dist_m = self.o_expand_th_m

        # expand before sampled node
        expand_num = math.floor(expand_dist_m / self.o_path_reso_m)
        for _ in range(expand_num):
            new_node.o_x_m += self.o_path_reso_m * math.cos(angle_rad)
            new_node.o_y_m += self.o_path_reso_m * math.sin(angle_rad)
            new_node.o_x_path.append(new_node.o_x_m)
            new_node.o_y_path.append(new_node.o_y_m)
        
        # expand from expanded node to sampled node
        dist_m, _ = self.calculate_distance_angle(new_node, a_to)
        if dist_m <= self.o_path_reso_m:
            new_node.o_x_path.append(a_to.o_x_m)
            new_node.o_y_path.append(a_to.o_y_m)
            new_node.o_x_m = a_to.o_x_m
            new_node.o_y_m = a_to.o_y_m
        
        # expanded node's parent node
        new_node.o_parent_node = a_from

        return new_node
    
    def find_near_nodes(self, a_node):
        node_num = len(self.o_node_list) + 1
        radius = self.o_connect_dist * math.sqrt(math.log(node_num)/node_num)
        radius = min(radius, self.o_expand_th_m)
        dist_list = [(node.o_x_m - a_node.o_x_m)**2 + (node.o_y_m - a_node.o_y_m)**2 for node in self.o_node_list]
        near_index_list = [dist_list.index(dist) for dist in dist_list if dist <= radius**2]
        return near_index_list
    
    def select_parent(self, a_node, a_index_list):
        # there is no near node
        if not a_index_list:
            return None
        
        # search minimum cost in near nodes
        cost_list = []
        for idx in a_index_list:
            near_node = self.o_node_list[idx]
            

    def append_inside_safe_node(self, a_node):
        if a_node is not None:
            if self.o_map.is_inside(a_node.o_x_m, a_node.o_y_m) \
                and self.o_map.is_safe(a_node.o_x_path, a_node.o_y_path):
                near_index_list = self.find_near_nodes(a_node)

                self.o_node_list.append(a_node)
    
    def calculate_dist_to_goal(self, a_x_m, a_y_m):
        diff_x_m = a_x_m - self.o_goal.o_x_m
        diff_y_m = a_y_m - self.o_goal.o_y_m
        return math.hypot(diff_x_m, diff_y_m)
    
    def find_final_path(self, a_goal_index):
        self.o_final_path = [[self.o_goal.o_x_m, self.o_goal.o_y_m]]
        node = self.o_node_list[a_goal_index]
        while node.o_parent_node is not None:
            self.o_final_path.append([node.o_x_m, node.o_y_m])
            node = node.o_parent_node
        self.o_final_path.append([node.o_x_m, node.o_y_m])

    def draw_searched_node(self, a_sampled_node):
        if a_sampled_node is not None:
            self.plot_samp_node.set_data(a_sampled_node.o_x_m, a_sampled_node.o_y_m)
        
        for node in self.o_node_list:
            if node is not None:
                self.o_axes.plot(node.o_x_path, node.o_y_path, "-c")

        plt.pause(0.001)
    
    def draw_final_path(self):
        if len(self.o_final_path) == 0:
            print("Failed planning path..")
        else:
            self.o_axes.plot([x_m for (x_m, y_m) in self.o_final_path],
                             [y_m for (x_m, y_m) in self.o_final_path],
                             "-r")
    
    def search_path(self):
        self.o_node_list = [self.o_start]

        for i in range(self.o_max_iter):
            rand_samp_node = self.random_sampling_node()

            nearest_node = self.get_nearest_node(rand_samp_node)

            new_node = self.expand_node(nearest_node, rand_samp_node)

            # calculate new node's cost from nearest node
            new_node.o_cost = nearest_node.o_cost + math.hypot(new_node.o_x_m - nearest_node.o_x_m,
                                                               new_node.o_y_m - nearest_node.o_y_m)

            self.append_inside_safe_node(new_node)

            if show_plot and i % 5 == 0:
                self.draw_searched_node(rand_samp_node)
            
            if self.calculate_dist_to_goal(self.o_node_list[-1].o_x_m,
                                           self.o_node_list[-1].o_y_m) <= self.o_expand_th_m:
                final_node = self.expand_node(self.o_node_list[-1], self.o_goal)
                if self.o_map.is_safe(final_node.o_x_path, final_node.o_y_path):
                    self.find_final_path(len(self.o_node_list)-1)
                    break


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
    
    # generate RRT instance
    rrts = RRTStar(ax, om, MIN_SAMPLING_AREA_M, MAX_SAMPLING_AREA_M)
    rrts.search_path()

    # only when show plot flag is true, show output graph
    # when unit test is executed, this flag become false
    # and the graph is not shown
    if show_plot:
        rrts.draw_final_path()
        plt.show()

    return True


# execute main process
if __name__ == "__main__":
    main()
