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

print(CubicSpline2D)
