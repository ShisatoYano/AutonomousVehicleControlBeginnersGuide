# 4. Path Tracking
This chapter explains the design and implementation of the path tracking controllers used in this project.

The documents in this chapter are:

## 4.1 [Stanley Controller](/doc/4_path_tracking/4_1_stanley_controller.md)
Covers the Stanley steering controller, which corrects heading error and cross-track error to keep the front axle on a reference course.

## 4.2 [MPC Controller](/doc/4_path_tracking/4_2_mpc_controller.md)
Covers the model predictive controller, which solves a finite-horizon optimization problem for steering and acceleration at every time step.

## 4.3 [MPPI Controller](/doc/4_path_tracking/4_3_mppi_controller.md)
Covers the model predictive path integral controller, which samples control sequences, rolls them forward, and combines them by cost.
