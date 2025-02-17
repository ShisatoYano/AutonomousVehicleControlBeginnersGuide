## 3.2 LiDAR model
In this section, a simple 2D LiDAR model class are implemented for generating a 2D point cloud data. The obstacles in the simulation world can be observed with this LiDAR model.  

### 3.2.1 Sensor parameters
A parameters class, SensorParameters is implemented here. This class has the following parameters for a sensor installation and observation.  

* Longitudinal installation position on a vehicle coordinate system[m]
* Lateral installation position on a vehicle coordinate system[m]
* Yaw angle of installation pose on a vehicle coordinate system[deg]
* Minimum sensing range[m]
* Maximum sensing range[m]
* Resolution of sensing angle[deg]
* Scale of an angle standard deviation
* Rate of a distance standard deviation

SensorParameters class is implemented as follow. [XYArray class](/doc/2_vehicle_model/2_vehicle_model.md) is imported in the class for representing a x-y vector of the position of the sensor. The above parameters can be given to the constructor as arguments.  

[sensor_parameters.py](/src/components/sensors/sensor_parameters.py)  
```python
"""
sensor_parameters.py

Author: Shisato Yano
"""

import numpy as np
import sys
from pathlib import Path

sys.path.append(str(Path(__file__).absolute().parent) + "/../array")
from xy_array import XYArray


class SensorParameters:
    """
    Parameters class for sensor
    """

    def __init__(self, lon_m=0.0, lat_m=0.0, yaw_deg=0.0, min_m=0.5, max_m=40, reso_deg=2.0,
                 angle_std_scale=0.01, dist_std_rate=0.005):
        """
        Constructor
        lon_m: longitudinal installation position on vehicle coordinate system[m]
        lat_m: lateral installation position on vehicle coordinate system[m]
        yaw_deg: yaw angle of installation position on vehicle coordinate system[deg]
        min_m: minimum sensing range[m]
        max_m: maximum sensing range[m]
        reso_deg: resolution of sensing angle[deg]
        angle_std_scale: scale of angle's standard deviation
        dist_std_rate: rate of distance's standard deviation
        """
        
        self.INST_LON_M = lon_m
        self.INST_LAT_M = lat_m
        self.INST_YAW_RAD = np.deg2rad(yaw_deg)

        self.MIN_RANGE_M = min_m
        self.MAX_RANGE_M = max_m

        self.RESO_RAD = np.deg2rad(reso_deg)

        self.ANGLE_STD_SCALE = angle_std_scale
        self.DIST_STD_RATE = dist_std_rate

        self.inst_pos_array = XYArray(np.array([[self.INST_LON_M], [self.INST_LAT_M]]))
        self.global_x_m = None
        self.global_y_m = None
```

And then, some member methods, calculate_global_pos, get_global_x_m, get_global_y_m and  draw_pos are implemented. calculate_global_pos method is used for transforming the position and pose of the sensor from the vehicle coordinate system to the global coordinate system based on the state of the vehicle.  
```python
    def calculate_global_pos(self, state):
        """
        Function to calculate sensor's installation position on global coordinate system
        state: vehicle's state object
        """

        pose = state.x_y_yaw()
        transformed_array = self.inst_pos_array.homogeneous_transformation(pose[0, 0], pose[1, 0], pose[2, 0])
        
        self.global_x_m = transformed_array.get_x_data()
        self.global_y_m = transformed_array.get_y_data()
```

get_global_x_m and get_global_y_m are used for getting the global position of the sensor.  
```python
    def get_global_x_m(self):
        """
        Getter of sensor's x installation position on global coordinate system
        """

        return self.global_x_m[0]
    
    def get_global_y_m(self):
        """
        Getter of sensor's y installation position on global coordinate system
        """

        return self.global_y_m[0]
```

draw_pos is used for visualizing the position of the sensor in the simulation world. In this method, the position of the sensor is transformed into the global position and visualized.  
```python
    def draw_pos(self, axes, elems, state):
        """
        Function to draw sensor's installation position on vehicle
        axes: axes object of figure
        elems: list of plot object
        state: vehicle state object
        """

        self.calculate_global_pos(state)
        pos_plot, = axes.plot(self.global_x_m, self.global_y_m, marker='.', color='b')
        elems.append(pos_plot)
```

### 3.2.2 Homogeneous transformation matrix
For transforming the generated point cloud, some utility methods of transformation matrix. These methods can process homogeneous 2D transformation and rotation.  
[matrix_lib.py](/src/components/common/matrix_lib.py)  
```python
"""
matrix_lib.py

Author: Shisato Yano
"""

import numpy as np
from math import sin, cos


def hom_mat_33(x, y, yaw):
    """
    Homogeneous transformation matrix 3x3
    x: x direction translation[m]
    y: y direction translation[m]
    yaw: yaw direction rotation[rad]
    """

    cos_yaw, sin_yaw = cos(yaw), sin(yaw)

    return np.array([[cos_yaw, -sin_yaw, x],
                     [sin_yaw, cos_yaw, y],
                     [0.0, 0.0, 1.0]])


def rot_mat_22(yaw):
    cos_yaw, sin_yaw = cos(yaw), sin(yaw)

    return np.array([[cos_yaw, -sin_yaw],
                     [sin_yaw, cos_yaw]])
```

### 3.2.3 Scan point
ScanPoint class is implemented to represent a scan point in a point cloud from LiDAR. [XYArray class](/doc/2_vehicle_model/2_vehicle_model.md) is imported in the class for representing a x-y vector of the position of the scan point. The scan point object has the following data.  

* Distance from the LiDAR to the scan point[m]
* Horizontal angle from the LiDAR to the scan point[rad]
* X-Y position array of the scan point[m]
* Trasformed x coordinate of the scan point on a specific coordinate system
* Trasformed y coordinate of the scan point on a specific coordinate system

The constructor of ScanPoint class is implemented as follow. A distance, horizontal angle, x coordinate and y coordinate are given as arguments.  
[scan_point.py](/src/components/sensors/lidar/scan_point.py)  
```python
"""
scan_point.py

Author: Shisato Yano
"""

import numpy as np
import sys
from pathlib import Path

sys.path.append(str(Path(__file__).absolute().parent) + "/../../array")
sys.path.append(str(Path(__file__).absolute().parent) + "/../../common")
from xy_array import XYArray
from matrix_lib import hom_mat_33


class ScanPoint:
    """
    Scan point of sensor class includes each sensing data
    """
    
    def __init__(self, distance_m, angle_rad, x_m, y_m):
        """
        Constructor
        distance_m: sensed distance data[m]
        angle_rad: sensed angle data[rad]
        x_m: sensed point's x coordinate data[m]
        y_m: sensed point's y coordinate data[m]
        """
        
        self.distance_m = distance_m
        self.angle_rad = angle_rad
        self.point_array = XYArray(np.array([[x_m], [y_m]]))
        self.transformed_x = None
        self.transformed_y = None
```

Then, some member methods for accessing or visualizing the data. The getter methods for a dimension of x-y array, distance and x-y array of point are implemented as follow.  
```python
    def get_dimension(self):
        """
        Return point's x-y array data's dimension value
        """

        return self.point_array.get_dimension()
    
    def get_distance_m(self):
        """
        Return point's distance data[m]
        """
        
        return self.distance_m
    
    def get_point_array(self):
        """
        Return point's x-y array data
        Type is ndarray object
        """

        return self.point_array.get_data()
```

To transform the scan point data, the following functions are implemented. These functions are used to get the transformed scan point by other class.  
```python
    def get_transformed_data(self, sensor_lon, sensor_lat, sensor_yaw,
                             vehicle_x, vehicle_y, vehicle_yaw):
        """
        Return transformed x-y array data based on specific coordinate system
        Type is ndarray object
        sensor_lon: longitudinal position of sensor on vehicle coordinate[m]
        sensor_lat: lateral position of sensor on vehicle coordinate[m]
        sensor_yaw: yaw angle of sensor on vehicle coordinate[rad]
        vehicle_x: x position of vehicle on global coordinate[m]
        vehicle_y: y position of vehicle on global coordinate[m]
        vehicle_yaw: yaw angle of vehicle on global coordinate[rad]
        """

        # transformation matrix on sensor coordinate
        point_xy = self.point_array.get_data()
        sensor_tf = hom_mat_33(point_xy[0, 0], point_xy[1, 0], 0.0)

        # transformation matrix on vehicle coordinate
        vehicle_tf = hom_mat_33(sensor_lon, sensor_lat, sensor_yaw)

        # transformation matrix on global coordinate
        global_tf = hom_mat_33(vehicle_x, vehicle_y, vehicle_yaw)

        # homegeneous transformation from sensor to global coordinate
        transformed_points_matrix = global_tf @ vehicle_tf @ sensor_tf

        return transformed_points_matrix[0, 2], transformed_points_matrix[1, 2]
    
    def calculate_transformed_point(self, sensor_lon, sensor_lat, sensor_yaw,
                                    vehicle_x, vehicle_y, vehicle_yaw):
        """
        Function to calculate transformed x-y point based on specific coordinate system
        sensor_lon: longitudinal position of sensor on vehicle coordinate[m]
        sensor_lat: lateral position of sensor on vehicle coordinate[m]
        sensor_yaw: yaw angle of sensor on vehicle coordinate[rad]
        vehicle_x: x position of vehicle on global coordinate[m]
        vehicle_y: y position of vehicle on global coordinate[m]
        vehicle_yaw: yaw angle of vehicle on global coordinate[rad]
        """
        
        self.transformed_x, self.transformed_y = \
            self.get_transformed_data(sensor_lon, sensor_lat, sensor_yaw,
                                      vehicle_x, vehicle_y, vehicle_yaw)
```

Finally, a function to draw the scan point is implemented as follow.  
```python
    def draw(self, axes, elems):
        """
        Function to draw scan point's x-y coordinate data
        axes: Axes object of figure
        elems: List of plot objects
        """

        if self.transformed_x and self.transformed_y:
            point_plot, = axes.plot(self.transformed_x, self.transformed_y, marker='.', color='b')
            elems.append(point_plot)
```

### 3.2.4 Omni Directional LiDAR
Omni directional LiDAR class is implemented to generate the point cloud by importing [Scan point class](#323-scan-point). This class has the following data.  

* List of obstacle objects
* Sensor parameters object
* Size of distance database at each angles
* Maximum value of distance database
* List of delta values list for interpolating obstacle contour

The constructor of this class is implemented as follow. The list of obstacles objects and sensor parameters object are given as the arguments. And then, the above member data is initialized.  
```python
"""
omni_directional_lidar.py

Author: Shisato Yano
"""

import numpy as np
from math import atan2, sin, cos
from scipy.stats import norm

from scan_point import ScanPoint

class OmniDirectionalLidar:
    """
    Sensing simulation class with Omni directional LiDAR
    """
    
    def __init__(self, obst_list, params):
        """
        Constructor
        obst_list: List of Obstacle objects
        params: Sensor parameters object
        """
        
        self.obst_list = obst_list
        self.params = params
        self.DIST_DB_SIZE = int(np.floor((np.pi * 2.0) / self.params.RESO_RAD)) + 1
        self.MAX_DB_VALUE = float("inf")
        self.DELTA_LIST = np.arange(0.0, 1.0, 0.008)
        self.latest_point_cloud = []
```

The installation position of this LiDAR need to be computed for transformation of the coordinate of the point cloud. The following function to compute the installation position on global coordinate system is implemented.  
```python
    def install(self, state):
        """
        Function to calculate installed position on global coordinate
        state: Vehicle's state object 
        """
        
        self.params.calculate_global_pos(state)
```

The obstacles within the coverage of LiDAR can be observed. The private function, _visible(self, distance_m) is used to check the obstacle is within the coverage.  
```python
    def _visible(self, distance_m):
        """
        Private function to check object is visible according to sensing distance
        distance_m: Sensing distance[m]
        """
        
        return (self.params.MIN_RANGE_M <= distance_m <= self.params.MAX_RANGE_M)
```

For normalizing an angle of scan point data, the following private functions are implemented.  
```python
    def _normalize_angle_until_2pi(self, angle_rad):
        """
        Private function to normalize sensing angle between 0 and 360 deg
        angle_rad: Sensing angle[rad]
        """

        if 0.0 > angle_rad: return (angle_rad + np.pi * 2.0)
        else: return angle_rad
    
    def _normalize_angle_pi_2_pi(self, angle_rad):
        """
        Private function to normalize sensing angle between -180 and 180 deg
        angle_rad: Sensing angle[rad]
        """

        if angle_rad > np.pi: return (angle_rad - np.pi * 2.0)
        else: return angle_rad
```

The above normalization functions are used to compute a scan point data and a point cloud data. A function to compute a point cloud with ray casting algorithm is implemented considering occlusion.  
```python
    def _ray_casting_filter(self, distance_list, angle_list, state):
        """
        Private function to filter point cloud by Ray casting
        distance_list: List of sensing distance[m]
        angle_list: List of sensing angle[rad]
        """

        point_cloud = []
        dist_db = [self.MAX_DB_VALUE for _ in range(self.DIST_DB_SIZE)]

        for i in range(len(angle_list)):
            normalized_angle_2pi = self._normalize_angle_until_2pi(angle_list[i])
            angle_id = int(round(normalized_angle_2pi / self.params.RESO_RAD)) % self.DIST_DB_SIZE
            if dist_db[angle_id] > distance_list[i]:
                dist_db[angle_id] = distance_list[i]
        
        for i in range(len(dist_db)):
            angle_rad = i * self.params.RESO_RAD
            angle_pi_2_pi = self._normalize_angle_pi_2_pi(angle_rad)
            distance_m = dist_db[i]
            if (distance_m != self.MAX_DB_VALUE) and self._visible(distance_m):
                angle_with_noise = norm.rvs(loc=angle_pi_2_pi, scale=self.params.ANGLE_STD_SCALE)
                dist_with_noise = norm.rvs(loc=distance_m, scale=self.params.DIST_STD_RATE*distance_m)
                x_m = dist_with_noise * cos(angle_with_noise)
                y_m = dist_with_noise * sin(angle_with_noise)
                point = ScanPoint(dist_with_noise, angle_with_noise, x_m, y_m)
                vehicle_pose = state.x_y_yaw()
                point.calculate_transformed_point(self.params.INST_LON_M, self.params.INST_LAT_M, self.params.INST_YAW_RAD,
                                                  vehicle_pose[0, 0], vehicle_pose[1, 0], vehicle_pose[2, 0])
                point_cloud.append(point)
        
        self.latest_point_cloud = point_cloud
```

For generating a scan point data, the following two functions need to be implemented. By using these functions, a line between two vertexes of the obstacle can be interpolated with multiple points. The coordinate of each points can be used as the scan points.  
```python
    def _interpolate(self, x_1, x_2, delta):
        """
        Private function to interpolate between two values
        x_1: value 1
        x_2: value 2
        delta: resolution between value 1 and 2
        """

        return ((1.0 - delta) * x_1 + delta * x_2)

    def _calculate_contour_xy(self, vertex_x, vertex_y):
        """
        Private function to calculate contour coordinates x-y
        vertex_x: List of vertex's x coordinate
        vertex_y: List of vertex's y coordinate
        """

        contour_x, contour_y = [], []
        len_vertex = len(vertex_x)

        for i in range(len_vertex - 1):
            contour_x.extend([self._interpolate(vertex_x[i], vertex_x[i+1], delta) for delta in self.DELTA_LIST])
            contour_y.extend([self._interpolate(vertex_y[i], vertex_y[i+1], delta) for delta in self.DELTA_LIST])
        
        contour_x.extend([self._interpolate(vertex_x[len_vertex-1], vertex_x[1], delta) for delta in self.DELTA_LIST])
        contour_y.extend([self._interpolate(vertex_y[len_vertex-1], vertex_y[1], delta) for delta in self.DELTA_LIST])

        return contour_x, contour_y
```