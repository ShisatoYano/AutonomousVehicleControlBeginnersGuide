"""
cubic_spline_course.py

Author: Shisato Yano
"""

# import path setting
import sys
from pathlib import Path

abs_dir_path = str(Path(__file__).absolute().parent)
relative_path = "/../../../components/"

sys.path.append(abs_dir_path + relative_path + "course/cubic_spline_course")

#import component modules
from cubic_spline_2d import CubicSpline2D


class CubicSplineCourse:
    """
    Course generation class by Cubic spline interpolation
    """

    def __init__(self, x_ref_points, y_ref_points, target_speed_kmph, color='r'):
        """
        Constructor
        x_ref_points: Array of reference points in x axis
        y_ref_points: Array of reference points in y axis
        target_speed_kmph: Target speed[km/h] driving on generated course
        color: Color of drawing course points
        """
        
        pass
