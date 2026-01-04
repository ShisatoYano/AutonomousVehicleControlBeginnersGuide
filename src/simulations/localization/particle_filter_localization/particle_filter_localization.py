"""
particle_filter_localization.py

Author: Sahruday Patti

This simulation demonstrates vehicle localization using a Particle Filter.
The Particle Filter is a Monte Carlo method that represents the belief
about the robot's state using a set of weighted samples (particles).

- Configurable number of particles (default: 500)
- Multiple resampling strategies: systematic, multinomial, low variance
"""

# import path setting
import sys
from pathlib import Path

abs_dir_path = str(Path(__file__).absolute().parent)
relative_path = "/../../../components/"

sys.path.append(abs_dir_path + relative_path + "visualization")
sys.path.append(abs_dir_path + relative_path + "state")
sys.path.append(abs_dir_path + relative_path + "vehicle")
sys.path.append(abs_dir_path + relative_path + "course/cubic_spline_course")
sys.path.append(abs_dir_path + relative_path + "control/pure_pursuit")
sys.path.append(abs_dir_path + relative_path + "sensors")
sys.path.append(abs_dir_path + relative_path + "sensors/gnss")
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
from particle_filter_localizer import ParticleFilterLocalizer


# flag to show plot figure
# when executed as unit test, this flag is set as false
show_plot = True


def main():
    """
    Main process function
    
    Demonstrates Particle Filter localization with:
    - 500 particles (configurable)
    - Systematic resampling (can be changed)
    - Pure pursuit path tracking controller
    - GNSS sensor with noise
    """
    
    # set simulation parameters
    x_lim, y_lim = MinMax(-5, 55), MinMax(-20, 25)
    vis = GlobalXYVisualizer(x_lim, y_lim, TimeParameters(span_sec=30))

    # create course data instance
    course = CubicSplineCourse([0.0, 10.0, 25, 40, 50],
                               [0.0, 4, -12, 20, -13],
                               20)
    vis.add_object(course)

    # create vehicle's spec instance
    spec = VehicleSpecification(area_size=20.0)
    
    # create vehicle's state instance
    state = State(color='b')
    
    # create controller instance
    pure_pursuit = PurePursuitController(spec, course, color='m')

    # create vehicle instance
    # set state, spec, controller, sensors and localizer instances as arguments
    gnss = Sensors(gnss=Gnss(x_noise_std=1.0, y_noise_std=1.0))
    
    # Create Particle Filter localizer
    # - num_particles: Number of particles (default 500)
    # - resampling_method: 'systematic', 'multinomial', or 'low_variance'
    # - color: Color for drawing particles and uncertainty ellipse
    pf = ParticleFilterLocalizer(
        num_particles=500,
        resampling_method='systematic',
        color='r'
    )
    
    vehicle = FourWheelsVehicle(state, spec, controller=pure_pursuit, 
                                 sensors=gnss, localizer=pf)
    vis.add_object(vehicle)

    # plot figure is not shown when executed as unit test
    if not show_plot: vis.not_show_plot()

    # show plot figure
    vis.draw()


# execute main process
if __name__ == "__main__":
    main()

