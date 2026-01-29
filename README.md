# AutonomousVehicleControlBeginnersGuide
[![Linux_CI](https://github.com/ShisatoYano/AutonomousDrivingSamplePrograms/actions/workflows/Linux_CI.yml/badge.svg)](https://github.com/ShisatoYano/AutonomousDrivingSamplePrograms/actions/workflows/Linux_CI.yml) [![Windows_CI](https://github.com/ShisatoYano/AutonomousDrivingSamplePrograms/actions/workflows/Windows_CI.yml/badge.svg)](https://github.com/ShisatoYano/AutonomousDrivingSamplePrograms/actions/workflows/Windows_CI.yml) [![MacOS_CI](https://github.com/ShisatoYano/AutonomousDrivingSamplePrograms/actions/workflows/MacOS_CI.yml/badge.svg)](https://github.com/ShisatoYano/AutonomousDrivingSamplePrograms/actions/workflows/MacOS_CI.yml) [![CodeFactor](https://www.codefactor.io/repository/github/shisatoyano/autonomousvehiclecontrolbeginnersguide/badge)](https://www.codefactor.io/repository/github/shisatoyano/autonomousvehiclecontrolbeginnersguide)  

Python sample codes and documents about Autonomous vehicle control algorithm. This project can be used as a technical guide book to study the algorithms and the software architectures for beginners.  

![](src/simulations/mapping/ndt_map_construction/ndt_map_construction.gif)  


## Table of Contents
* [What is this?](#what-is-this)
* [Goal of this project](#goal-of-this-project)
* [Requirements](#requirements)
* [How to use](#how-to-use)
* [Examples of Simulation](#examples-of-simulation)
    * [Localization](#localization)
        * [Extended Kalman Filter Localization](#extended-kalman-filter-localization)
        * [Unscented Kalman Filter Localization](#unscented-kalman-filter-localization)
        * [Particle Filter Localization](#particle-filter-localization)
    * [Mapping](#mapping)
        * [Binary Occupancy Grid Map Construction](#binary-occupancy-grid-map-construction)
        * [Cost Map Construction](#cost-map-construction)
        * [NDT Map Construction](#ndt-map-construction)
    * [Path Planning](#path-planning)
        * [A*](#a)
        * [Bidirectional A*](#bidirectional-a)
        * [Hybrid A*](#hybrid-a)
        * [Dijkstra](#dijkstra)
        * [RRT](#rrt)
    * [Path Tracking](#path-tracking)
        * [Pure pursuit Path Tracking](#pure-pursuit-path-tracking)
        * [Rear wheel feedback Path Tracking](#rear-wheel-feedback-path-tracking)
        * [LQR(Linear Quadratic Regulator) Path Tracking](#lqrlinear-quadratic-regulator-path-tracking)
        * [Stanley steering control Path tracking](#stanley-steering-control-path-tracking)
    * [Perception](#perception)
        * [Rectangle fitting Detection](#rectangle-fitting-detection)
        * [Sensor's Extrinsic Parameters Estimation](#sensors-extrinsic-parameters-estimation)
* [Documents](#documents)
* [License](#license)
* [Use Case](#use-case)
* [Contribution](#contribution)
* [Author](#author)


## What is this?
This is a sample codes collections about Autonomous vehicle control algorithm. Each source codes are implemented with Python to help your understanding. You can fork this repository and use for studying, education or work freely.  


## Goal of this project
I want to release my own technical book about Autonomous Vehicle algorithms in the future. The book will include all of codes and documents in this repository as contents.  


## Requirements
Please satisfy with the following requirements on native or VM Linux in advance.  
For running each sample codes:  
* [Python 3.13.x](https://www.python.org/)
* [Matplotlib](https://matplotlib.org/)
* [NumPy](https://numpy.org/)
* [SciPy](https://scipy.org/)

For development:
* [pytest](https://docs.pytest.org/en/7.4.x/) (for unit tests)
* [pytest-cov](https://github.com/pytest-dev/pytest-cov) (for coverage measurement)

For setting up the environment with Docker:
* [VS Code](https://code.visualstudio.com/)
* [Docker](https://www.docker.com/)


## How to use
1. Clone this repository  
    ```bash
    $ git clone https://github.com/ShisatoYano/AutonomousVehicleControlBeginnersGuide
    ```

2. Set up the environment for running each codes
    * Set up with Docker on WSL:
        * Before cloning thi repo, [install Docker](https://docs.docker.com/desktop/install/linux-install/) in advance
        * Clone this repo following the above Step 1
        * Open this repo's folder by VSCode
        * [Create Dev Container](https://code.visualstudio.com/docs/devcontainers/create-dev-container)
        * And then, all required libraries are installed automatically
3. Execute unit tests to confirm the environment were installed successfully
    ```bash
    $ . run_test_suites.sh
    ```
4. Execute a python script at src/simulations directory
    * For example, when you want to execute localization simulation of Extended Kalman Filter:
        ```bash
        $ python src/simulations/localization/extended_kalman_filter_localization/extended_kalman_filter_localization.py
        ```
5. Add star to this repository if you like it!!


## Examples of Simulation
### Localization
#### Extended Kalman Filter Localization
![](src/simulations/localization/extended_kalman_filter_localization/extended_kalman_filter_localization.gif)  
#### Unscented Kalman Filter Localization
![](src/simulations/localization/unscented_kalman_filter_localization/unscented_kalman_filter_localization.gif)  
#### Particle Filter Localization
![](src/simulations/localization/particle_filter_localization/particle_filter_localization.gif)  
#### EKF(Blue) vs UKF(Cyan) vs PF(Lime)
![](src/simulations/localization/ekf_ukf_pf_comparison/ekf_ukf_pf_comparison.gif)    
### Mapping
#### Binary Occupancy Grid Map Construction
![](src/simulations/mapping/binary_grid_map_construction/binary_grid_map_construction.gif)  
#### Cost Map Construction
![](src/simulations/mapping/cost_grid_map_construction/cost_grid_map_construction.gif)  
#### NDT Map Construction
![](src/simulations/mapping/ndt_map_construction/ndt_map_construction.gif)  
### Path Planning
#### A*
Planning  
![](src/simulations/path_planning/astar_path_planning/astar_search.gif)  
Navigation  
![](src/simulations/path_planning/astar_path_planning/astar_navigate.gif)  
#### Bidirectional A*
Planning  
![](src/simulations/path_planning/astar_bidirectional_path_planning/astar_bidirectional_search.gif)  
Navigation  
![](src/simulations/path_planning/astar_bidirectional_path_planning/astar_bidirectional_navigate.gif)  
#### Hybrid A*
Planning  
![](src/simulations/path_planning/astar_hybrid_path_planning/astar_hybrid_search.gif)  
Navigation  
![](src/simulations/path_planning/astar_hybrid_path_planning/astar_hybrid_navigate.gif)  
#### Dijkstra
Planning(Reduce frames by sampling every nth node to prevent memory exhaustion)  
![](src/simulations/path_planning/dijkstra_path_planning/dijkstra_search.gif)  
Navigation  
![](src/simulations/path_planning/dijkstra_path_planning/dijkstra_navigate.gif)  
#### RRT
Planning  
![](src/simulations/path_planning/rrt_path_planning/rrt_search.gif)  
Navigation  
![](src/simulations/path_planning/rrt_path_planning/rrt_navigate.gif)  
### Path Tracking
#### Pure pursuit Path Tracking
![](src/simulations/path_tracking/pure_pursuit_path_tracking/pure_pursuit_path_tracking.gif)  
#### Rear wheel feedback Path Tracking
![](src/simulations//path_tracking/rear_wheel_feedback_tracking/rear_wheel_feedback_tracking.gif)  
#### LQR(Linear Quadratic Regulator) Path Tracking
![](src/simulations/path_tracking/lqr_path_tracking/lqr_path_tracking.gif)  
#### Stanley steering control Path Tracking
![](src/simulations/path_tracking/stanley_path_tracking/stanley_path_tracking.gif)  
### Perception
#### Rectangle fitting Detection
![](src/simulations/perception/point_cloud_rectangle_fitting/point_cloud_rectangle_fitting.gif)  
#### Sensor's Extrinsic Parameters Estimation
Estimation by Unscented Kalman Filter  
![](src/simulations/perception/sensor_auto_calibration/sensor_auto_calibration.gif)  


## Documents
Design documents of each Python programs are prepared here. The documents are still not completed. They have been being updated. If you found any problems in them, please tell me by creating an issue.  
[Documents link](/doc/DESIGN_DOCUMENT.md)  


## License
MIT  


## Use Case
I started this project to study an algorithm and software development for Autonomous Vehicle system by myself. You can also use this repo for your own studying, education, researching and development.  

If this project helps your task, please let me know by creating a issue.  
Any paper, animation, video as your output, always welcome!! It will encourage me to continue this project.  

Your comment and output is added to [this list of user comments](/USERS_COMMENTS.md).  


## Contribution
Any contribution by creating an issue or sending a pull request is welcome!! Please check [this document about how to contribute](/HOWTOCONTRIBUTE.md).  


## Author
[Shisato Yano](https://github.com/ShisatoYano)  
