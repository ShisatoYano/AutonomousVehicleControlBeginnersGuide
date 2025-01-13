"""
rear_wheel_feedback_controller.py

Author: Shisato Yano
"""

#import path setting
import sys
from pathlib import Path
from math import sin, cos, atan2

abs_dir_path = str(Path(__file__).absolute().parent)
relative_path = "/../../../components/"

sys.path.append(abs_dir_path + relative_path + "control/speed_profile")

#import component modules
from trapezoidal_speed_profile import TrapezoidalSpeedProfile
