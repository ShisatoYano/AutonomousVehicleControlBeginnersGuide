"""
ekf_ukf_pf_comparison.py

Author: Sahruday Patti

This simulation compares three localization methods:
1. Extended Kalman Filter (EKF) - Blue trajectory, Red ellipse
2. Unscented Kalman Filter (UKF) - Cyan trajectory, Orange ellipse  
3. Particle Filter (PF) - Green trajectory, Purple particles/ellipse

All three filters use the same:
- Motion model (kinematic bicycle model)
- Observation model (GNSS position)
- Noise parameters

This allows for a fair comparison of their localization performance.
"""

# import path setting
import sys
from pathlib import Path
import matplotlib.lines as mlines

abs_dir_path = str(Path(__file__).absolute().parent)
relative_path = "/../../../components/"

sys.path.append(abs_dir_path + relative_path + "visualization")
sys.path.append(abs_dir_path + relative_path + "state")
sys.path.append(abs_dir_path + relative_path + "vehicle")
sys.path.append(abs_dir_path + relative_path + "course/cubic_spline_course")
sys.path.append(abs_dir_path + relative_path + "control/pure_pursuit")
sys.path.append(abs_dir_path + relative_path + "sensors")
sys.path.append(abs_dir_path + relative_path + "sensors/gnss")
sys.path.append(abs_dir_path + relative_path + "localization/kalman_filter")
sys.path.append(abs_dir_path + relative_path + "localization/particle_filter")


# import component modules
from global_xy_visualizer import GlobalXYVisualizer
from min_max import MinMax
from time_parameters import TimeParameters
from vehicle_specification import VehicleSpecification
from state import State
from four_wheels_vehicle import FourWheelsVehicle
from cubic_spline_course import CubicSplineCourse
from pure_pursuit_controller import PurePursuitController
from sensors import Sensors
from gnss import Gnss
from extended_kalman_filter_localizer import ExtendedKalmanFilterLocalizer
from unscented_kalman_filter_localizer import UnscentedKalmanFilterLocalizer
from particle_filter_localizer import ParticleFilterLocalizer


class ComparisonLegend:
    """
    Helper class to draw a legend for the EKF vs UKF vs PF comparison.
    Follows the same interface pattern as other visualization objects.
    """
    
    def __init__(self):
        self.legend_added = False
    
    def draw(self, axes, elems):
        """
        Draw legend on the plot.
        Only adds legend once since it persists across frames.
        """
        if not self.legend_added:
            # Create legend handles
            ekf_handle = mlines.Line2D([], [], color='b', marker='.', linestyle='None',
                                       markersize=8, label='EKF (blue, red ellipse)')
            ukf_handle = mlines.Line2D([], [], color='c', marker='.', linestyle='None',
                                       markersize=8, label='UKF (cyan, orange ellipse)')
            pf_handle = mlines.Line2D([], [], color='lime', marker='.', linestyle='None',
                                      markersize=8, label='PF (lime, purple particles)')
            gnss_handle = mlines.Line2D([], [], color='g', marker='.', linestyle='None',
                                        markersize=8, label='GNSS observations')
            
            axes.legend(handles=[ekf_handle, ukf_handle, pf_handle, gnss_handle],
                       loc='upper left', fontsize=9, framealpha=0.9)
            self.legend_added = True


# flag to show plot figure
# when executed as unit test, this flag is set as false
show_plot = True


def main():
    """
    Main process function
    
    Comparison between EKF, UKF, and Particle Filter localization.

    All filters use identical noise parameters for fair comparison.
    """
    
    # set simulation parameters
    x_lim, y_lim = MinMax(-5, 55), MinMax(-20, 25)
    vis = GlobalXYVisualizer(x_lim, y_lim, TimeParameters(span_sec=30))

    # create course data instance (trajectory)
    course = CubicSplineCourse([0.0, 10.0, 25, 40, 50],
                               [0.0, 4, -12, 20, -13],
                               20)
    vis.add_object(course)

    # create vehicle's spec instance
    spec = VehicleSpecification(area_size=20.0)
    
    # Common sensor with same noise for all vehicles
    gnss = Sensors(gnss=Gnss(x_noise_std=1.0, y_noise_std=1.0, color='g'))
    
    # ========== EKF Vehicle (Blue trajectory, Red ellipse) ==========
    ekf_state = State(color='b', x_m=0.0, y_m=0.0, yaw_rad=0.0, speed_mps=0.0)
    ekf_controller = PurePursuitController(spec, course, color='m')
    ekf_localizer = ExtendedKalmanFilterLocalizer(color='r')
    ekf_vehicle = FourWheelsVehicle(ekf_state, spec, controller=ekf_controller, 
                                    sensors=gnss, localizer=ekf_localizer)
    vis.add_object(ekf_vehicle)
    
    # ========== UKF Vehicle (Cyan trajectory, Orange ellipse) ==========
    ukf_state = State(color='c', x_m=0.0, y_m=0.0, yaw_rad=0.0, speed_mps=0.0)
    ukf_controller = PurePursuitController(spec, course, color='m')
    ukf_localizer = UnscentedKalmanFilterLocalizer(color='orange')
    ukf_vehicle = FourWheelsVehicle(ukf_state, spec, controller=ukf_controller, 
                                     sensors=gnss, localizer=ukf_localizer)
    vis.add_object(ukf_vehicle)
    
    # ========== PF Vehicle (Green trajectory, Purple particles/ellipse) ==========
    pf_state = State(color='lime', x_m=0.0, y_m=0.0, yaw_rad=0.0, speed_mps=0.0)
    pf_controller = PurePursuitController(spec, course, color='m')
    pf_localizer = ParticleFilterLocalizer(
        num_particles=500,
        resampling_method='systematic',
        color='purple'
    )
    pf_vehicle = FourWheelsVehicle(pf_state, spec, controller=pf_controller, 
                                    sensors=gnss, localizer=pf_localizer)
    vis.add_object(pf_vehicle)
    
    # ========== Add Legend ==========
    legend = ComparisonLegend()
    vis.add_object(legend)

    # plot figure is not shown when executed as unit test
    if not show_plot: vis.not_show_plot()

    # show plot figure
    vis.draw()


# execute main process
if __name__ == "__main__":
    main()

