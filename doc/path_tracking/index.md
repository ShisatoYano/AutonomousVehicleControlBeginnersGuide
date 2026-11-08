# Chapter 4: Following a Course

```{note}
This chapter is being written. Sections 4.1, 4.2 and 4.4 have their theory; their code walkthroughs, experiments and exercises are planned.
```

Path tracking algorithms in Python: generate a smooth course with cubic splines, and steer the vehicle along it with the Pure Pursuit, Stanley and rear wheel feedback controllers, which use the model of [Chapter 3](/doc/vehicle_model/index.md) as geometry.

## Sections

- [4.1 Course generation with cubic splines](/doc/path_tracking/cubic_spline.md): [`cubic_spline`](/src/simulations/course/cubic_spline)
- [4.2 Pure Pursuit](/doc/path_tracking/pure_pursuit.md): [`pure_pursuit_path_tracking`](/src/simulations/path_tracking/pure_pursuit_path_tracking), [`adaptive_pure_pursuit_path_tracking`](/src/simulations/path_tracking/adaptive_pure_pursuit_path_tracking)
- [4.3 Stanley](/doc/path_tracking/stanley.md): [`stanley_path_tracking`](/src/simulations/path_tracking/stanley_path_tracking)
- [4.4 Rear wheel feedback](/doc/path_tracking/rear_wheel_feedback.md): [`rear_wheel_feedback_tracking`](/src/simulations/path_tracking/rear_wheel_feedback_tracking)

## The system so far

The vehicle follows a course with a geometric controller, using its true state.

Shown today by `pure_pursuit_path_tracking`.

## Before this chapter

[Chapter 3](/doc/vehicle_model/index.md).
