# Chapter 7: Building a Map

```{note}
This chapter is planned. This page lists what it will cover and the simulations that already exist for it.
```

Mapping from LiDAR point clouds in Python: occupancy grid maps, cost maps, potential field maps and NDT maps that a planner can use.

## Sections

- 7.1 Binary occupancy grid map: [`binary_grid_map_construction`](/src/simulations/mapping/binary_grid_map_construction)
- 7.2 Cost map: [`cost_grid_map_construction`](/src/simulations/mapping/cost_grid_map_construction)
- 7.3 Potential field map: [`potential_field_map_construction`](/src/simulations/mapping/potential_field_map_construction)
- 7.4 NDT map: [`ndt_map_construction`](/src/simulations/mapping/ndt_map_construction)

## The system so far

The vehicle builds a map of its surroundings while it drives.

Shown today by `ndt_map_construction`.

## Before this chapter

[Chapter 5](/doc/sensors/index.md), [Chapter 6](/doc/localization/index.md).
