# Chapter 9: Finding Objects: Perception

```{note}
This chapter is planned. This page lists what it will cover and the simulations that already exist for it.
```

Object detection from a LiDAR point cloud in Python: search for neighboring points with a kd-tree, then fit rectangles to the clusters (L-shape fitting).

## Sections

- 9.1 Nearest neighbor search with a kd-tree: [`point_cloud_search`](/src/simulations/perception/point_cloud_search)
- 9.2 Rectangle fitting: [`point_cloud_rectangle_fitting`](/src/simulations/perception/point_cloud_rectangle_fitting)

## The system so far

The vehicle detects the objects around it as rectangles.

Shown today by `point_cloud_rectangle_fitting`.

## Before this chapter

[Chapter 5](/doc/sensors/index.md).
