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