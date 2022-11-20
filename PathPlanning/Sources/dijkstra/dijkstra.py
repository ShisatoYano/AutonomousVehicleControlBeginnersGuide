"""
Path Planning sample program by Dijkstra.

Author: Shisato Yano
"""

import matplotlib.pyplot as plt
import math

# parameters
START_X_M = -5.0 # start point
START_Y_M = -5.0
GOAL_X_M = 50.0 # goal point
GOAL_Y_M = 50.0
GRID_SIZE_M = 2.0
VEHICLE_SIZE_HALF_M = 1.5

# flag to switch showing output graph or not
show_plot = True


class ObstacleGridMap:
    def __init__(self, a_start_x_m, a_start_y_m, a_goal_x_m, a_goal_y_m,
                 a_grid_size_m, a_vehicle_size_half_m):
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
    
    def set_obstacles_position(self):
        self.o_obst_x = []
        self.o_obst_y = []
        for i in range(-10, 60): self.o_obst_x.append(i), self.o_obst_y.append(-10.0)
        for i in range(-10, 60): self.o_obst_x.append(60.0), self.o_obst_y.append(i)
        for i in range(-10, 61): self.o_obst_x.append(i), self.o_obst_y.append(60.0)
        for i in range(-10, 60): self.o_obst_x.append(-10.0), self.o_obst_y.append(i)
        for i in range(-10, 40): self.o_obst_x.append(20.0), self.o_obst_y.append(i)
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
            x = self.calculate_xy_position_from_index(idx_x, self.o_min_x_m)
            for idx_y in range(self.o_gird_width_y):
                y = self.calculate_xy_position_from_index(idx_y, self.o_min_y_m)
                for obst_x, obst_y in zip(self.o_obst_x, self.o_obst_y):
                    diff_xy = math.hypot(obst_x - x, obst_y - y)
                    if diff_xy <= self.o_vehicle_size_half_m:
                        self.o_obstacle_map[idx_x][idx_y] = True
                        break
    
    def calculate_xy_position_from_index(self, a_index, a_min_position):
        return a_index * self.o_grid_size_m + a_min_position        

    def draw(self):
        plt.plot(self.o_obst_x, self.o_obst_y, ".k")
        plt.plot(self.o_start_x_m, self.o_start_y_m, "og")
        plt.plot(self.o_goal_x_m, self.o_goal_y_m, "xb")
        plt.grid(True)
        plt.axis("equal")


class Dijkstra:
    def __init__(self, a_map):
        self.o_map = a_map
        self.define_motion_model()
    
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


def main():
    print(__file__ + " start!!")

    ogm = ObstacleGridMap(a_start_x_m=START_X_M, a_start_y_m=START_Y_M, 
                          a_goal_x_m=GOAL_X_M, a_goal_y_m=GOAL_Y_M, 
                          a_grid_size_m=GRID_SIZE_M, 
                          a_vehicle_size_half_m=VEHICLE_SIZE_HALF_M)
    
    dijkstra = Dijkstra(a_map=ogm)
    
    # only when show plot flag is true, show output graph
    # when unit test is executed, this flag become false
    # and the graph is not shown
    if show_plot: plt.show()

    return True


# execute main process
if __name__ == "__main__":
    main()
