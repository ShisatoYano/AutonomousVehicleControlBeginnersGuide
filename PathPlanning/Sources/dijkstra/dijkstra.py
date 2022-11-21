"""
Path Planning sample program by Dijkstra.

Author: Shisato Yano
"""

import matplotlib.pyplot as plt
import math

# parameters
START_X_M = 10.0 # start point
START_Y_M = 10.0
GOAL_X_M = 50.0 # goal point
GOAL_Y_M = 50.0
GRID_SIZE_M = 1.0
VEHICLE_SIZE_HALF_M = 2.0

# flag to switch showing output graph or not
show_plot = True


class ObstacleGridMap:
    def __init__(self, a_axes, a_start_x_m, a_start_y_m, a_goal_x_m, 
                 a_goal_y_m, a_grid_size_m, a_vehicle_size_half_m):
        # set parameters
        self.o_start_x_m = a_start_x_m
        self.o_start_y_m = a_start_y_m
        self.o_goal_x_m = a_goal_x_m
        self.o_goal_y_m = a_goal_y_m
        self.o_grid_size_m = a_grid_size_m
        self.o_vehicle_size_half_m = a_vehicle_size_half_m

        self.set_obstacles_position()
        self.calculate_map_range()
        self.calculate_map_grid_width()
        self.generate_obstacle_map()

        self.plot_obstacles, = a_axes.plot(self.o_obst_x, self.o_obst_y, ".k")
        self.plot_start, = a_axes.plot(self.o_start_x_m, self.o_start_y_m, "og")
        self.plot_goal, = a_axes.plot(self.o_goal_x_m, self.o_goal_y_m, "xb")
    
    def set_obstacles_position(self):
        self.o_obst_x = []
        self.o_obst_y = []
        for i in range(0, 60): self.o_obst_x.append(i), self.o_obst_y.append(0.0)
        for i in range(0, 60): self.o_obst_x.append(60.0), self.o_obst_y.append(i)
        for i in range(0, 61): self.o_obst_x.append(i), self.o_obst_y.append(60.0)
        for i in range(0, 60): self.o_obst_x.append(0.0), self.o_obst_y.append(i)
        for i in range(0, 40): self.o_obst_x.append(20.0), self.o_obst_y.append(i)
        for i in range(0, 40): self.o_obst_x.append(40.0), self.o_obst_y.append(60.0-i)
    
    def calculate_map_range(self):
        self.o_min_x_m = round(min(self.o_obst_x))
        self.o_min_y_m = round(min(self.o_obst_y))
        self.o_max_x_m = round(max(self.o_obst_x))
        self.o_max_y_m = round(max(self.o_obst_y))
        print("Min X[m]:", self.o_min_x_m)
        print("Min Y[m]:", self.o_min_y_m)
        print("Max X[m]:", self.o_max_x_m)
        print("Max Y[m]:", self.o_max_y_m)

    def calculate_map_grid_width(self):
        self.o_gird_width_x = round((self.o_max_x_m - self.o_min_x_m) / self.o_grid_size_m)
        self.o_gird_width_y = round((self.o_max_y_m - self.o_min_y_m) / self.o_grid_size_m)
        print("Grid Width X:", self.o_gird_width_x)
        print("Grid Width Y:", self.o_gird_width_y)
    
    def generate_obstacle_map(self):
        self.o_obstacle_map = [[False for _ in range(self.o_gird_width_y)]
                               for _ in range(self.o_gird_width_x)]
        for idx_x in range(self.o_gird_width_x):
            x = self.calculate_x_position_from_index(idx_x)
            for idx_y in range(self.o_gird_width_y):
                y = self.calculate_y_position_from_index(idx_y)
                for obst_x, obst_y in zip(self.o_obst_x, self.o_obst_y):
                    diff_xy = math.hypot(obst_x - x, obst_y - y)
                    if diff_xy <= self.o_vehicle_size_half_m:
                        self.o_obstacle_map[idx_x][idx_y] = True
                        break
    
    def calculate_x_position_from_index(self, a_idx_x):
        return a_idx_x * self.o_grid_size_m + self.o_min_x_m
    
    def calculate_y_position_from_index(self, a_idx_y):
        return a_idx_y * self.o_grid_size_m + self.o_min_y_m

    def calculate_x_index_from_position(self, a_pos_x):
        return round((a_pos_x - self.o_min_x_m) / self.o_grid_size_m)
    
    def calculate_y_index_from_position(self, a_pos_y):
        return round((a_pos_y - self.o_min_y_m) / self.o_grid_size_m)
    
    def calculate_vector_index(self, a_idx_x, a_idx_y):
        return a_idx_y * self.o_gird_width_x + a_idx_x 

    def start_x_index(self):
        return self.calculate_x_index_from_position(self.o_start_x_m)

    def start_y_index(self):
        return self.calculate_y_index_from_position(self.o_start_y_m)

    def start_vector_index(self):
        return self.calculate_vector_index(self.start_x_index(),
                                           self.start_y_index())

    def goal_x_index(self):
        return self.calculate_x_index_from_position(self.o_goal_x_m)

    def goal_y_index(self):
        return self.calculate_x_index_from_position(self.o_goal_y_m)

    def goal_vector_index(self):
        return self.calculate_vector_index(self.goal_x_index(), 
                                           self.goal_y_index())
    
    def verify_index(self, a_idx_x, a_idx_y):
        x = self.calculate_x_position_from_index(a_idx_x)
        y = self.calculate_y_position_from_index(a_idx_y)

        if x < self.o_min_x_m: return False
        if y < self.o_min_y_m: return False
        
        if x >= self.o_max_x_m: return False
        if y >= self.o_max_y_m: return False

        if self.o_obstacle_map[a_idx_x][a_idx_y]: return False

        return True


class Dijkstra:
    def __init__(self, a_axes, a_map):
        self.o_map = a_map
        self.o_open_set = dict()
        self.o_closed_set = dict()
        self.define_motion_model()
    
    class Node:
        def __init__(self, a_idx_x, a_idx_y, a_cost, a_idx_prnt):
            self.o_idx_x = a_idx_x
            self.o_idx_y = a_idx_y
            self.o_cost = a_cost
            self.o_idx_prnt = a_idx_prnt
        
        def __str__(self):
            return "Idx X:" + str(self.o_idx_x) + ", Idx Y:" + str(self.o_idx_y) + ", Cost:" + \
                str(self.o_cost) + ", Parent Idx:" + str(self.o_idx_prnt)

    def define_motion_model(self):
        # move direction x, move direction y, move cost(distance)
        self.motion_model = [[1, 0, 1], # right
                             [0, 1, 1], # up
                             [-1, 0, 1], # left
                             [0, -1, 1], # down
                             [-1, -1, math.sqrt(2)], # lower left
                             [-1, 1, math.sqrt(2)], # upper left
                             [1, -1, math.sqrt(2)], # lower right
                             [1, 1, math.sqrt(2)]] # upper right
    
    def search_path(self):
        start_node = self.Node(self.o_map.start_x_index(),
                               self.o_map.start_y_index(), 
                               0.0, -1)
        
        goal_node = self.Node(self.o_map.goal_x_index(),
                              self.o_map.goal_y_index(), 
                              0.0, -1)
        
        self.o_open_set[self.o_map.start_vector_index()] = start_node

        print("Start node info: ", start_node, "Vector Idx:", self.o_map.start_vector_index())
        print("Goal node info: ", goal_node, "Vector Idx:", self.o_map.goal_vector_index())

        while 1:
            current_idx, current_node = self.select_min_cost_node_from_open_set()

            if show_plot: self.draw_searched_node(current_node)
            
            if self.found_goal(current_node, goal_node):
                print("Found Goal!!")
                goal_node.o_idx_prnt = current_node.o_idx_prnt
                goal_node.o_cost == current_node.o_cost
                break

            self.remove_current_node_from_open_set(current_idx)

            self.move_current_node_to_closed_set(current_node, current_idx)

            self.search_neighbor_node(current_node, current_idx)

        x_list, y_list = self.find_final_path(goal_node)

        return x_list, y_list
    
    def select_min_cost_node_from_open_set(self):
        current_idx = min(self.o_open_set, key=lambda o: self.o_open_set[o].o_cost)
        current_node = self.o_open_set[current_idx]
        
        return current_idx, current_node
    
    def draw_searched_node(self, a_crnt_node):
        plt.plot(self.o_map.calculate_x_position_from_index(a_crnt_node.o_idx_x),
                 self.o_map.calculate_y_position_from_index(a_crnt_node.o_idx_y),
                 "xc")
        
        if len(self.o_closed_set.keys()) % 10 == 0: plt.pause(0.001)
    
    def remove_current_node_from_open_set(self, a_crnt_idx):
        del self.o_open_set[a_crnt_idx]
    
    def move_current_node_to_closed_set(self, a_crnt_node, a_crnt_idx):
        self.o_closed_set[a_crnt_idx] = a_crnt_node
    
    def found_goal(self, a_crnt_node, a_goal_node):
        return (a_crnt_node.o_idx_x == a_goal_node.o_idx_x) and \
            (a_crnt_node.o_idx_y == a_goal_node.o_idx_y)
    
    def search_neighbor_node(self, a_crnt_node, a_crnt_idx):
        for move_dist_x, move_dist_y, move_cost in self.motion_model:
            neighbor_node = self.Node(a_crnt_node.o_idx_x + move_dist_x, 
                                      a_crnt_node.o_idx_y + move_dist_y, 
                                      a_crnt_node.o_cost + move_cost, 
                                      a_crnt_idx)
            neighbor_idx = self.o_map.calculate_vector_index(neighbor_node.o_idx_x, 
                                                             neighbor_node.o_idx_y)
            
            if neighbor_idx in self.o_closed_set:
                continue

            if not self.o_map.verify_index(neighbor_node.o_idx_x, neighbor_node.o_idx_y):
                continue

            if neighbor_idx not in self.o_open_set:
                    self.o_open_set[neighbor_idx] = neighbor_node
            else:
                if self.o_open_set[neighbor_idx].o_cost >= neighbor_node.o_cost:
                    self.o_open_set[neighbor_idx] = neighbor_node

    def find_final_path(self, a_goal_node):
        x_list = [self.o_map.calculate_x_position_from_index(a_goal_node.o_idx_x)]
        y_list = [self.o_map.calculate_y_position_from_index(a_goal_node.o_idx_y)]

        parent_idx = a_goal_node.o_idx_prnt
        while parent_idx != -1:
            parent_node = self.o_closed_set[parent_idx]
            x_list.append(self.o_map.calculate_x_position_from_index(parent_node.o_idx_x))
            y_list.append(self.o_map.calculate_y_position_from_index(parent_node.o_idx_y))
            parent_idx = parent_node.o_idx_prnt
        
        return x_list, y_list
    
    def draw_final_path(self, a_x_list, a_y_list):
        plt.plot(a_x_list, a_y_list, "-r")


def main():
    print(__file__ + " start!!")

    # initialize plot
    ax = plt.subplot(1, 1, 1)
    ax.set_xlabel("X[m]")
    ax.set_ylabel("Y[m]")
    ax.set_aspect("equal")
    ax.grid(True)

    # generate map instance
    ogm = ObstacleGridMap(ax, START_X_M, START_Y_M, GOAL_X_M, 
                          GOAL_Y_M, GRID_SIZE_M, VEHICLE_SIZE_HALF_M)
    
    dijkstra = Dijkstra(ax, ogm)
    x_list, y_list = dijkstra.search_path()
    
    # only when show plot flag is true, show output graph
    # when unit test is executed, this flag become false
    # and the graph is not shown
    if show_plot:
        dijkstra.draw_final_path(x_list, y_list)
        plt.show()

    return True


# execute main process
if __name__ == "__main__":
    main()
