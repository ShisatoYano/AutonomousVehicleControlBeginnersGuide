## 3.1 Obstacle
This section explains the obstacle models used by the sensor simulations. The repository provides two modules:

- [obstacle.py](/src/components/obstacle/obstacle.py): Represents one obstacle (static or dynamic).
- [obstacle_list.py](/src/components/obstacle/obstacle_list.py): Manages multiple obstacles.

### 3.1.1 Rectangle Obstacle
The `Obstacle` class stores its own state and geometric shape. The constructor takes a [State class](/doc/2_vehicle_model/2_vehicle_model.md) instance and shape/motion parameters.

```python
class Obstacle:
    def __init__(self, state, accel_mps2=0.0, yaw_rate_rps=0.0,
                 length_m=2.0, width_m=2.0):
        self.state = state
        self.accel_mps2 = accel_mps2
        self.yaw_rate_rps = yaw_rate_rps
        self.length_m = length_m
        self.width_m = width_m
```

Notes:

- `accel_mps2` and `yaw_rate_rps` control obstacle motion.
- When both are `0.0`, the obstacle behaves as a static object.
- `width_m` is treated as half-width in the class contour definition.

The update function advances obstacle motion using the state model:

```python
def update(self, time_s):
    self.state.update(self.accel_mps2, self.yaw_rate_rps, time_s)
```

The draw function transforms the rectangle contour from obstacle coordinates to global coordinates and plots it:

```python
def draw(self, axes, elems):
    x_m = self.state.get_x_m()
    y_m = self.state.get_y_m()
    yaw_rad = self.state.get_yaw_rad()

    transformed_array = self.array.homogeneous_transformation(x_m, y_m, yaw_rad)
    obstacle_plot, = axes.plot(transformed_array.get_x_data(),
                               transformed_array.get_y_data(),
                               lw=1.0, color='k', ls='-')
    elems.append(obstacle_plot)
```

The `vertex_xy()` method returns transformed corner coordinates for downstream modules such as LiDAR point-cloud generation.

### 3.1.2 Obstacles List
`ObstacleList` is a small container class for multiple obstacles:

```python
class ObstacleList:
    def __init__(self):
        self.list = []

    def add_obstacle(self, obstacle):
        self.list.append(obstacle)
```

It also provides batch `update()` and `draw()` calls, plus `get_list()` for other modules (for example, LiDAR).

### 3.1.3 Visualization
A sample script for obstacle visualization is provided:

- [visualize_obstacle.py](/doc/3_sensor_models/visualize_obstacle.py)

This script creates one static obstacle and two dynamic obstacles, then draws them in the simulation world together with a vehicle model.

The output example is:

![](/doc/3_sensor_models/visualize_obstacle.gif)
