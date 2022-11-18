"""
Path Planning sample program by Dijkstra.

Author: Shisato Yano
"""

# parameters
START_X_M = -5.0 # start point
START_Y_M = -5.0
GOAL_X_M = 50.0 # goal point
GOAL_Y_M = 50.0
GRID_SIZE_M = 2.0
VEHICLE_SIZE_HALF_M = 1.5


class ObstacleGridMap:
    def __init__(self, start_x_m, start_y_m, goal_x_m, goal_y_m,
                 grid_size_m, vehicle_size_half_m):
        self.obst_x = []
        self.obst_y = []
    
    def set_obstacles_positions(self):
        for i in range(-10, 60): self.obst_x.append(i), self.obst_y.append(-10.0)
        for i in range(-10, 60): self.obst_x.append(60.0), self.obst_y.append(i)
        for i in range(-10, 61): self.obst_x.append(i), self.obst_y.append(60.0)
        for i in range(-10, 60): self.obst_x.append(-10.0), self.obst_y.append(i)
        for i in range(-10, 40): self.obst_x.append(20.0), self.obst_y.append(i)
        for i in range(-10, 40): self.obst_x.append(40.0), self.obst_y.append(60.0-i)


def main():
    print(__file__ + " start!!")

    return True


# execute main process
if __name__ == "__main__":
    main()
