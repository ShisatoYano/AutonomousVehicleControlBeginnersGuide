# Chapter 8: Planning a Route

```{note}
This chapter is planned. This page lists what it will cover and the simulations that already exist for it.
```

Path planning algorithms for autonomous vehicles in Python: find a route from the start to the goal on a map with Dijkstra, A*, Hybrid A*, D*, RRT, RRT*, PRM and optimization-based and learning-based planners.

## Sections

- 8.1 Graph search on a grid: Dijkstra, A*, bidirectional A*: [`dijkstra_path_planning`](/src/simulations/path_planning/dijkstra_path_planning), [`astar_path_planning`](/src/simulations/path_planning/astar_path_planning), [`astar_bidirectional_path_planning`](/src/simulations/path_planning/astar_bidirectional_path_planning)
- 8.2 Search with the vehicle's motion: Hybrid A*: [`astar_hybrid_path_planning`](/src/simulations/path_planning/astar_hybrid_path_planning)
- 8.3 Replanning: D*: [`dstar_path_planning`](/src/simulations/path_planning/dstar_path_planning)
- 8.4 Sampling-based planning: PRM, RRT, RRT*, bidirectional RRT*, Informed RRT*: [`prm_path_planning`](/src/simulations/path_planning/prm_path_planning), [`rrt_path_planning`](/src/simulations/path_planning/rrt_path_planning), [`rrt_star_path_planning`](/src/simulations/path_planning/rrt_star_path_planning), [`rrt_star_bidirectional_path_planning`](/src/simulations/path_planning/rrt_star_bidirectional_path_planning), [`informed_rrt_star_path_planning`](/src/simulations/path_planning/informed_rrt_star_path_planning)
- 8.5 Optimization and smoothing: gradient descent, elastic bands: [`gradient_path_planning`](/src/simulations/path_planning/gradient_path_planning), [`elastic_bands_path_planning`](/src/simulations/path_planning/elastic_bands_path_planning)
- 8.6 Nature-inspired and learning-based planning: GA, PSO, ACO, Q-learning: [`ga_path_planning`](/src/simulations/path_planning/ga_path_planning), [`pso_path_planning`](/src/simulations/path_planning/pso_path_planning), [`aco_path_planning`](/src/simulations/path_planning/aco_path_planning), [`q_learning_path_planning`](/src/simulations/path_planning/q_learning_path_planning)

## The system so far

A planner computes the course that the vehicle follows. Today the planners run on their own maps; the full loop comes with the navigation stack ([#115](https://github.com/ShisatoYano/AutonomousVehicleControlBeginnersGuide/issues/115)).

Shown today by `astar_path_planning`.

## Before this chapter

[Chapter 3](/doc/vehicle_model/index.md) (for Hybrid A*), [Chapter 7](/doc/mapping/index.md).
