"""
body.py

Author: Shisato Yano
"""


class Body:
    """
    Vehicle Body class
    """

    def __init__(self, array):
        """
        Constructor
        spec: vehicle specification object
        """

        self.array = array

    def draw(self, axes, spec, pose):
        transformed_array = self.array.homogeneous_transformation(pose[0, 0], pose[1, 0], pose[2, 0])
        return axes.plot(transformed_array[0, :], 
                         transformed_array[1, :], 
                         lw=spec.line_w, 
                         color=spec.color, 
                         ls=spec.line_type)
