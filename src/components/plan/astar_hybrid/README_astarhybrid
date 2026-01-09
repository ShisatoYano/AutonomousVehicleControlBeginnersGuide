# Hybrid A* Path Planner for Kinematic Vehicles

This project implements a Hybrid A Path Planning algorithm* designed for vehicles with non-holonomic constraints (like a car). Unlike standard A* which is restricted to grid cells, this planner generates smooth, driveable paths by simulating actual vehicle steering primitives.
The base code for A* algorithm and beautiful visualization was adopted from https://github.com/ShisatoYano/AutonomousVehicleControlBeginnersGuide/blob/main/src/components/plan/astar/astar_path_planner.py


## Highlights
- Kinematic Constraints: Uses a bicycle model with a minimum turning radius ($R$) to ensure the path can be followed by a real vehicle.
-  Continuous State Space: Operates in $SE(2)$ space $(x, y, \theta)$, allowing for sub-pixel precision in vehicle positioning. 
- Discretized Search (3D Closed Set): To maintain efficiency, the search prunes nodes using a 3D grid, preventing infinite expansion in continuous space.
- Heading Awareness: The cost function and goal check incorporate heading (yaw), ensuring the vehicle arrives at the destination facing the correct direction.
- Grid-to-World Synchronization: Seamlessly integrates with standard 2D occupancy grids while performing high-fidelity continuous math.


## Brief Code explanation 
1. Motion Primitives: Instead of moving to the 8 immediate neighbors, the planner "drives" forward using three steering commands:

* Steer Full Left ($-1/R$)
* Straight ($0$)
* Steer Full Right ($+1/R$)

2. Heuristic Function: The heuristic guides the search using a combination of:

- Euclidean Distance: Straight-line distance to the target $(x, y)$.
- Angular Penalty: The shortest angular distance to the goal yaw, calculated using atan2(sin, cos) to handle the $180^\circ$ to $-180^\circ$ boundary.

3. Coordinate Systems: 

- World Coordinates: Used for simulation and kinematic math (floats in meters/radians).
- Grid Indices: Used for collision checking against the occupancy grid and for visualization (integers).

## Tuning Parameters
To adapt the planner for different vehicles or environments, you can modify the following parameters in the ```__init__``` method:

1. Kinematic Constraints

- ```self. R``` (Minimum Turning Radius): Defines how sharply the vehicle can turn. A larger $R$ creates wider turns, while a smaller $R$ allows for tighter maneuvers in cluttered spaces.
- ```self.motion_primitives```: This list determines the steering angles tested at each step. Adding more steering values can result in smoother paths, but increases computation time.
 
2. Search Performance

- ```self.resolution```: Inferred from the grid map. It defines the size of the "bins" for the discrete closed set. Smaller resolution improves accuracy but significantly increases the number of nodes explored.
- ```yaw_res```: The discretization of the heading (inside the search function). Narrower bins (e.g., $5^\circ$ vs $10^\circ$) force the planner to be more precise with orientation but make the search slower.

3. Cost and Heuristic Weights

- ```self.weight```: The primary A* weight. Increasing this makes the search more "greedy" (moving faster toward the goal), while decreasing it makes the search more "exhaustive" (finding a more optimal path).

- ```self.yaw_cost_weight```: Controls how much the planner prioritizes reaching the correct Goal Yaw.
    - High weight: The vehicle will start turning much earlier to align its heading with the goal.
    - Low weight: The vehicle will focus on reaching the $(x, y)$ coordinates first, potentially resulting in sharp "corrective" turns at the end. 

4. Goal Tolerance
- ```dist_tolerance```: How close the vehicle needs to be to the goal point in meters.
- ```yaw_tolerance```: The allowable heading error (in radians) at the goal. Tight tolerances ensure high precision but may take longer to solve.

## References:
1. Gentle introduction to Hybrid A star : https://medium.com/@junbs95/gentle-introduction-to-hybrid-a-star-9ce93c0d7869
2. Practical Search Techniques in Path Planning for Autonomous Driving: https://ai.stanford.edu/~ddolgov/papers/dolgov_gpp_stair08.pdf

## Simulations

![Path Search](https://github.com/user-attachments/assets/6b3f8c90-2c5c-453e-81c8-6240fa7810a8)

![Path Tracking with Pure Pursuit](https://github.com/user-attachments/assets/f1ab4ef8-9efa-4a7e-804d-edb1b0299683)

### Thanks!
Please let me know if you have questions.
