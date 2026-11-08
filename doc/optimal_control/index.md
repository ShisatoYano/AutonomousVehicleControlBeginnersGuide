# Chapter 10: Optimal Control

```{note}
This chapter is being written. Some sections already exist, the others are planned.
```

Linearize and discretize the model of [Chapter 3](/doc/vehicle_model/index.md), and use it to compute the controls that minimize a cost: as a linear model (LQR), as the constraints of an optimization (MPC), and to roll out sampled controls (MPPI).

## Sections

- 10.1 Linearization and discretization of the vehicle model
- 10.2 LQR: [`lqr_path_tracking`](/src/simulations/path_tracking/lqr_path_tracking)
- [10.3 Model predictive control (MPC)](/doc/optimal_control/mpc.md): [`mpc_path_tracking`](/src/simulations/path_tracking/mpc_path_tracking)
- [10.4 Model predictive path integral control (MPPI)](/doc/optimal_control/mppi.md): [`mppi_path_tracking`](/src/simulations/path_tracking/mppi_path_tracking)

## The system so far

The geometric controller of [Chapter 4](/doc/path_tracking/index.md) is swapped for an optimal controller, and the two are compared on the same course.

Shown today by `mpc_path_tracking`.

## Before this chapter

[Chapter 4](/doc/path_tracking/index.md), [Chapter 6](/doc/localization/index.md), Math background: linear algebra, linearization, optimization.
