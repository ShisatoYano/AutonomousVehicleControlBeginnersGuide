# Autonomous Vehicle Control Beginners Guide

Learn autonomous driving by building it: Python simulations and an open textbook where the system grows one stage at a time.

The book starts from a bare world and a vehicle that moves by given commands, and **adds one stage per chapter**: following a course, sensing, localization, mapping, planning, perception and optimal control, then driving among other vehicles and obstacles. Each chapter ends with a runnable "system so far", and the alternative algorithms of a stage are swapped into the same system and compared.

The book is a work in progress: most chapters are planned, and list the simulations that already exist for them. See the [documentation roadmap](https://github.com/ShisatoYano/AutonomousVehicleControlBeginnersGuide/issues/165) for the plan. Comments and ideas are welcome in the [Book](https://github.com/ShisatoYano/AutonomousVehicleControlBeginnersGuide/discussions/categories/book) category of Discussions.

```{toctree}
:caption: "Part I: The Base"
:maxdepth: 1

getting_started/index
simulator/index
vehicle_model/index
```

```{toctree}
:caption: "Part II: Building the System"
:maxdepth: 1

path_tracking/index
sensors/index
localization/index
mapping/index
path_planning/index
perception/index
optimal_control/index
```

```{toctree}
:caption: "Part III: Driving Among Others"
:maxdepth: 1

object_tracking/index
obstacle_avoidance/index
lane_change/index
dynamic_vehicle_model/index
```

```{toctree}
:caption: Appendix
:maxdepth: 1

math_background/index
NOTATION
EXAMPLES
ARCHITECTURE
WRITING_GUIDE
exercise_answers/index
```

```{toctree}
:caption: "Design documents (being moved into the chapters)"
:maxdepth: 1

4_path_tracking/4_path_tracking
```
