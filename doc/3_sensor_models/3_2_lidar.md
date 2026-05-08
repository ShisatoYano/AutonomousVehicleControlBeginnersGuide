## 3.2 LiDAR Model
This section explains the 2D LiDAR implementation used in the simulation. The LiDAR stack is composed of:

- [sensor_parameters.py](/src/components/sensors/sensor_parameters.py)
- [scan_point.py](/src/components/sensors/lidar/scan_point.py)
- [omni_directional_lidar.py](/src/components/sensors/lidar/omni_directional_lidar.py)
- [ukf_ext_params_calibrator.py](/src/components/sensors/lidar/ukf_ext_params_calibrator.py)

### 3.2.1 Sensor Parameters
`SensorParameters` stores installation parameters and LiDAR measurement parameters.

```python
class SensorParameters:
    def __init__(self, lon_m=0.0, lat_m=0.0, yaw_deg=0.0, min_m=0.5, max_m=40, reso_deg=2.0,
                 angle_std_scale=0.01, dist_std_rate=0.005, calibrator=None):
```

Key parameters:

- Installation pose on the vehicle coordinate system (`lon_m`, `lat_m`, `yaw_deg`)
- Sensing range and angular resolution (`min_m`, `max_m`, `reso_deg`)
- Noise model parameters (`angle_std_scale`, `dist_std_rate`)
- Optional calibration module (`calibrator`)

Main methods:

- `calculate_global_pos(state)`: Converts sensor installation position from vehicle coordinates to global coordinates.
- `calculate_sensor_odometry(state)`: Stores previous and current sensor poses as homogeneous transforms.
- `calibrate_extrinsic_params(vehicle_state)`: If a calibrator is attached, estimates extrinsic parameters using sensor and vehicle odometry.
- `draw_pos(axes, elems, state)`: Draws sensor position, and also draws calibration results when a calibrator exists.

### 3.2.2 Transformation Utilities
LiDAR point transformation uses homogeneous transformation matrices from:

- [matrix_lib.py](/src/components/common/matrix_lib.py)

`hom_mat_33(x, y, yaw)` is used to convert points from sensor coordinates to vehicle/global coordinates.

### 3.2.3 ScanPoint
`ScanPoint` represents one LiDAR return in the point cloud.

- distance and angle in sensor coordinates
- local point coordinates (`x_m`, `y_m`)
- transformed global coordinates used for drawing

```python
class ScanPoint:
    def __init__(self, distance_m, angle_rad, x_m, y_m):
        self.distance_m = distance_m
        self.angle_rad = angle_rad
        self.point_array = XYArray(np.array([[x_m], [y_m]]))
        self.transformed_x = None
        self.transformed_y = None
```

To transform coordinates, `ScanPoint` chains three transforms:

1. Point on sensor coordinates
2. Sensor pose on vehicle coordinates
3. Vehicle pose on global coordinates

This is implemented in `get_transformed_data(...)` and `calculate_transformed_point(...)`.

### 3.2.4 OmniDirectionalLidar
`OmniDirectionalLidar` generates a 2D point cloud from obstacle contours.

```python
class OmniDirectionalLidar:
    def __init__(self, obst_list, params):
        self.obst_list = obst_list
        self.params = params
        self.DIST_DB_SIZE = int(np.floor((np.pi * 2.0) / self.params.RESO_RAD)) + 1
        self.MAX_DB_VALUE = float("inf")
        self.DELTA_LIST = np.arange(0.0, 1.0, 0.008)
        self.latest_point_cloud = []
```

Main processing flow in `update(state)`:

1. Update sensor global position
2. Update sensor odometry
3. Run optional extrinsic calibration
4. Generate contour points for each obstacle
5. Convert contour points to range/angle measurements
6. Apply ray-casting filter (occlusion handling + nearest point per angle bin)
7. Inject distance/angle noise and generate `ScanPoint` objects

Current implementation:

```python
def update(self, state):
    self.params.calculate_global_pos(state)
    self.params.calculate_sensor_odometry(state)
    self.params.calibrate_extrinsic_params(state)

    distance_list, angle_list = [], []
    for obst in self.obst_list.get_list():
        vertex_x, vertex_y = obst.vertex_xy()
        contour_x, contour_y = self._calculate_contour_xy(vertex_x, vertex_y)
        for x, y in zip(contour_x, contour_y):
            diff_x = x - self.params.get_global_x_m()
            diff_y = y - self.params.get_global_y_m()
            distance_m = np.hypot(diff_x, diff_y)
            angle_rad = atan2(diff_y, diff_x) - state.get_yaw_rad()
            distance_list.append(distance_m)
            angle_list.append(angle_rad)

    self._ray_casting_filter(distance_list, angle_list, state)
```

`draw(axes, elems, state)` draws both LiDAR position and point cloud.

### 3.2.5 Extrinsic Parameter Auto Calibration (Optional)
`UkfExtParamsCalibrator` estimates LiDAR extrinsic parameters on the vehicle coordinate system:

- Longitudinal position
- Lateral position
- Yaw angle

Source:

- [ukf_ext_params_calibrator.py](/src/components/sensors/lidar/ukf_ext_params_calibrator.py)

The module uses an Unscented Kalman Filter (UKF):

- Predicts static extrinsic state via sigma points
- Predicts sensor odometry from vehicle odometry and candidate extrinsics
- Updates state using innovation between predicted and measured sensor odometry

When this calibrator is passed into `SensorParameters(calibrator=...)`, the calibration process runs automatically during LiDAR updates and the estimated parameter values are drawn in the simulation.

Example simulation:

- [sensor_auto_calibration.py](/src/simulations/perception/sensor_auto_calibration/sensor_auto_calibration.py)
- Output GIF: ![](/src/simulations/perception/sensor_auto_calibration/sensor_auto_calibration.gif)
