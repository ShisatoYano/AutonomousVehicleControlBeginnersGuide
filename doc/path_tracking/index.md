# Chapter 4: Following a Course

```{note}
This chapter is being written. Some sections already exist, the others are planned.
```

Generate a smooth course, and steer the vehicle along it with geometric controllers that use the model of [Chapter 3](/doc/vehicle_model/index.md) as geometry.

## Sections

- 4.1 Course generation with cubic splines: [`cubic_spline`](/src/simulations/course/cubic_spline)
- 4.2 Pure Pursuit: [`pure_pursuit_path_tracking`](/src/simulations/path_tracking/pure_pursuit_path_tracking), [`adaptive_pure_pursuit_path_tracking`](/src/simulations/path_tracking/adaptive_pure_pursuit_path_tracking)
- 4.3 Stanley: [`stanley_path_tracking`](/src/simulations/path_tracking/stanley_path_tracking)
- 4.4 Rear wheel feedback: [`rear_wheel_feedback_tracking`](/src/simulations/path_tracking/rear_wheel_feedback_tracking)

## The system so far

The vehicle follows a course with a geometric controller, using its true state.

Shown today by `pure_pursuit_path_tracking`.

## Before this chapter

[Chapter 3](/doc/vehicle_model/index.md).

## Existing material

These pages are being moved into this chapter ([#170](https://github.com/ShisatoYano/AutonomousVehicleControlBeginnersGuide/issues/170)):

- [4.1 Stanley Controller (already rewritten with the writing guide)](/doc/4_path_tracking/4_1_stanley_controller.md)
