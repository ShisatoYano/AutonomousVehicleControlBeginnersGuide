# Chapter 6: Knowing Where You Are: Localization

```{note}
This chapter is planned. This page lists what it will cover and the simulations that already exist for it.
```

Vehicle localization in Python with the Kalman filter, the extended Kalman filter (EKF), the unscented Kalman filter (UKF) and the particle filter. Estimate the vehicle's pose from noisy inputs and GNSS with these Bayes filters, using the motion model of [Chapter 3](/doc/vehicle_model/index.md) with noise. Then give the controller the estimate instead of the true state, and see how the tracking changes.

## Sections

- 6.1 The Kalman filter in one dimension: [`kalman_filter_1d`](/src/simulations/localization/kalman_filter_1d)
- 6.2 Extended Kalman filter: [`extended_kalman_filter_localization`](/src/simulations/localization/extended_kalman_filter_localization)
- 6.3 Unscented Kalman filter: [`unscented_kalman_filter_localization`](/src/simulations/localization/unscented_kalman_filter_localization)
- 6.4 Particle filter: [`particle_filter_localization`](/src/simulations/localization/particle_filter_localization)
- 6.5 Comparing the filters: [`ekf_vs_ukf_comparison`](/src/simulations/localization/ekf_vs_ukf_comparison), [`ekf_ukf_pf_comparison`](/src/simulations/localization/ekf_ukf_pf_comparison)
- 6.6 (Planned) EKF SLAM, [#64](https://github.com/ShisatoYano/AutonomousVehicleControlBeginnersGuide/issues/64)

## The system so far

The controller uses the estimated pose instead of the true state.

Shown today by `extended_kalman_filter_localization`.

## Before this chapter

[Chapter 4](/doc/path_tracking/index.md), [Chapter 5](/doc/sensors/index.md), Math background: probability.
