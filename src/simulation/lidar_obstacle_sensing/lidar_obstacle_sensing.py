"""
lidar_obstacle_sensing.py

Author: Shisato Yano
"""

import sys
from pathlib import Path

abs_dir_path = str(Path(__file__).absolute().parent)
sys.path.append(abs_dir_path + "/../../visualization")
sys.path.append(abs_dir_path + "/../../state")
sys.path.append(abs_dir_path + "/../../vehicle")

from global_xy_visualizer import GlobalXYVisualizer
from vehicle_specification import VehicleSpecification
from state import State
from state_history import StateHistory
from four_wheels_vehicle import FourWheelsVehicle


show_plot = True


def main():
    vis = GlobalXYVisualizer(x_min=-30, x_max=30, y_min=-30, y_max=30, time_span_s=20)

    # vehicle instance
    spec = VehicleSpecification()
    vehicle_state = State(0.0, 0.0, 0.0, 0.0)
    history = StateHistory([vehicle_state.get_x_m()], [vehicle_state.get_y_m()], spec.color)
    vehicle = FourWheelsVehicle(vehicle_state, history, spec,
                                draw_area_width=30.0)
    vis.add_object(vehicle)

    if not show_plot: vis.not_show_plot()

    vis.draw()


if __name__ == "__main__":
    main()
