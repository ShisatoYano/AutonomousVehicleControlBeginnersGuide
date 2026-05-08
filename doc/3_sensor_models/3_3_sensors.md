## 3.3 Sensors
This section explains the `Sensors` wrapper class, which manages installed sensor modules and provides a single interface to the vehicle model.

Source:

- [sensors.py](/src/components/sensors/sensors.py)

### 3.3.1 Constructor
The current constructor accepts both LiDAR and GNSS modules.

```python
class Sensors:
    def __init__(self, lidar=None, gnss=None):
        self.lidar = lidar
        self.gnss = gnss
```

- `lidar`: Object that provides LiDAR update/draw APIs
- `gnss`: Object that provides GNSS update/draw APIs

### 3.3.2 Installation
`install(state)` computes installation-dependent data for sensors that require it.

```python
def install(self, state):
    if self.lidar:
        self.lidar.install(state)
```

In the current implementation, LiDAR requires explicit installation. GNSS does not require a separate install step.

### 3.3.3 Update and Draw
`Sensors` updates and draws each installed sensor module.

```python
def update_data(self, state):
    if self.lidar:
        self.lidar.update(state)
    if self.gnss:
        self.gnss.update(state)


def draw_data(self, axes, elems, state):
    if self.lidar:
        self.lidar.draw(axes, elems, state)
    if self.gnss:
        self.gnss.draw(axes, elems)
```

### 3.3.4 Getters
The wrapper also provides helper getters used by other modules.

LiDAR getters:

- `get_point_cloud_from_lidar()`
- `get_lidar_global_x_m()`
- `get_lidar_global_y_m()`

GNSS getter:

- `get_xy_pos_from_gnss()`

```python
def get_xy_pos_from_gnss(self):
    if self.gnss:
        return self.gnss.get_xy_pos()
    else:
        return None
```

### 3.3.5 Vehicle Integration
`Sensors` is integrated into [FourWheelsVehicle](/src/components/vehicle/four_wheels_vehicle.py).

Current constructor:

```python
def __init__(self, state, spec, controller=None, sensors=None,
             detector=None, mapper=None, localizer=None, show_zoom=True):
```

Current update sequence in the vehicle class:

1. Update sensor data
2. Update detection data
3. Update map data
4. Update control command
5. Update localization/state

Because of this flow, LiDAR/GNSS data can be consumed by detector, mapper, and localizer modules in the same simulation loop.

### 3.3.6 GNSS Module
GNSS module source:

- [gnss.py](/src/components/sensors/gnss/gnss.py)

The module provides noisy `(x, y)` observations based on vehicle state:

- `observation_model(state)` extracts ideal position
- `update(state)` adds observation noise
- `draw(axes, elems)` visualizes observation history
- `get_xy_pos()` returns latest observation for localization modules

Example usage appears in:

- [extended_kalman_filter_localization.py](/src/simulations/localization/extended_kalman_filter_localization/extended_kalman_filter_localization.py)

### 3.3.7 LiDAR Sensing Simulation
A LiDAR sensing example is available here:

- [lidar_obstacle_sensing.py](/src/simulations/perception/lidar_obstacle_sensing/lidar_obstacle_sensing.py)

This simulation installs a LiDAR module through `Sensors(lidar=...)`, senses multiple obstacles, and visualizes point-cloud outputs.

![](/src/simulations/perception/lidar_obstacle_sensing/lidar_obstacle_sensing.gif)
