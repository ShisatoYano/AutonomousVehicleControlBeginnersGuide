# Chapter 10: Following a Course Optimally: Optimal Control

```{note}
This chapter is planned. This page lists what it will cover and the simulations that already exist for it.
```

Linearize and discretize the model of [Chapter 3](/doc/vehicle_model/index.md), and use it to compute the controls that minimize a cost: as a linear model (LQR), as the constraints of an optimization (MPC), and to roll out sampled controls (MPPI).

## Sections

- 10.1 Linearization and discretization of the vehicle model
- 10.2 LQR: [`lqr_path_tracking`](/src/simulations/path_tracking/lqr_path_tracking)
- 10.3 MPC: [`mpc_path_tracking`](/src/simulations/path_tracking/mpc_path_tracking)
- 10.4 MPPI: [`mppi_path_tracking`](/src/simulations/path_tracking/mppi_path_tracking)

## The system so far

The geometric controller of [Chapter 4](/doc/path_tracking/index.md) is swapped for an optimal controller, and the two are compared on the same course.

Shown today by `mpc_path_tracking`.

## Before this chapter

[Chapter 4](/doc/path_tracking/index.md), [Chapter 6](/doc/localization/index.md), Math background: linear algebra, linearization, optimization.

## Existing material

These pages are being moved into this chapter ([#170](https://github.com/ShisatoYano/AutonomousVehicleControlBeginnersGuide/issues/170)):

- [4.2 MPC Controller](/doc/4_path_tracking/4_2_mpc_controller.md)
- [4.3 MPPI Controller](/doc/4_path_tracking/4_3_mppi_controller.md)
