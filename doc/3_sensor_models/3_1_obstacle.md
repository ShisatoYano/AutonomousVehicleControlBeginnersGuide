## 3.1 Obstacle
In this section, Obstacle class and Landmark class are implemented. These class are used for locating the obstacles object in the world. The located obstacles can be observed by an sensor model.   

### 3.1.1 Rectangle obstacle
The obstacle class is implemented as follow. [State class implemented in Chapter 2](/doc/2_vehicle_model/2_vehicle_model.md) is given to the constructor for computing the position and pose of the obstacle. And then, [XYArray class](/doc/2_vehicle_model/2_vehicle_model.md) is also imported in the class for representing the shape as rectangle.  

[obstacle.py](/src/components/obstacle/obstacle.py)  
```python
"""
obstacle.py

Author: Shisato Yano
"""

import numpy as np
import sys
from pathlib import Path

sys.path.append(str(Path(__file__).absolute().parent) + "/../array")
from xy_array import XYArray


class Obstacle:
    """
    Obstacle's data and logic class
    """

    def __init__(self, state, accel_mps2=0.0, yaw_rate_rps=0.0,
                 length_m=2.0, width_m=2.0):
        """
        Constructor
        state: Obstacle's state object
        accel_mps2: acceleration input to move[m/s2]
        yaw_rate_rps: yaw rate input to move[rad/s]
        length_m: Obstacle's longitudinal size[m]
        width_m: Obstacle's half of lateral size[m]
        """

        self.state = state
        self.accel_mps2 = accel_mps2
        self.yaw_rate_rps = yaw_rate_rps

        contour = np.array([[length_m, -length_m, -length_m, length_m, length_m],
                            [width_m, width_m, -width_m, -width_m, width_m]])
        self.array = XYArray(contour)
```

The given arguments, accel_mps2 and yaw_rate_rps are used for making the obstacle moved as a dynamic obstacle. If these values are default 0.0, the obstacle is represented as a static obstacle. Additionally, length_m and width_m are used for setting the size of the obstacle.  