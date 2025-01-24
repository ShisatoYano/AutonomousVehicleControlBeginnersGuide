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

SensorParameters class is implemented as follow. [XYArray class](/doc/2_vehicle_model/2_vehicle_model.md) is imported in the class for representing a x-y vector of the position of the sensor.  

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