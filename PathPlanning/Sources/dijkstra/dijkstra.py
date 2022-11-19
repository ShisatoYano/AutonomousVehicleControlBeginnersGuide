"""
Path Planning sample program by Dijkstra.

Author: Shisato Yano
"""

import matplotlib.pyplot as plt

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
    
    def set_obstacles_position(self):
        self.o_obst_x = []
        self.o_obst_y = []
        for i in range(-10, 60): self.o_obst_x.append(i), self.o_obst_y.append(-10.0)
        for i in range(-10, 60): self.o_obst_x.append(60.0), self.o_obst_y.append(i)
        for i in range(-10, 61): self.o_obst_x.append(i), self.o_obst_y.append(60.0)
        for i in range(-10, 60): self.o_obst_x.append(-10.0), self.o_obst_y.append(i)
        for i in range(-10, 40): self.o_obst_x.append(20.0), self.o_obst_y.append(i)
        for i in range(0, 40): self.o_obst_x.append(40.0), self.o_obst_y.append(60.0-i)
    
    def draw(self):
        plt.plot(self.o_obst_x, self.o_obst_y, ".k")
        plt.plot(self.o_start_x_m, self.o_start_y_m, "og")
        plt.plot(self.o_goal_x_m, self.o_goal_y_m, "xb")
        plt.grid(True)
        plt.axis("equal")


def main():
    print(__file__ + " start!!")

    ogm = ObstacleGridMap(a_start_x_m=START_X_M, a_start_y_m=START_Y_M, 
                          a_goal_x_m=GOAL_X_M, a_goal_y_m=GOAL_Y_M, 
                          a_grid_size_m=GRID_SIZE_M, 
                          a_vehicle_size_half_m=VEHICLE_SIZE_HALF_M)
    
    ogm.draw()
    
    # only when show plot flag is true, show output graph
    # when unit test is executed, this flag become false
    # and the graph is not shown
    if show_plot: plt.show()

    return True


# execute main process
if __name__ == "__main__":
    main()
