# 1.2 Running a Simulation and the Tests

How to run the autonomous driving simulations in Python: start one with a single command, save its animation as a GIF, and check your environment with the test suite.

Every simulation is one Python script under `src/simulations/`, grouped by category (`localization`, `mapping`, `path_planning`, `path_tracking`, `perception`, `course`). You run the script you want from the repository root, and it shows an animation of the algorithm at work. After this section, you can run any simulation in the repository and confirm that your setup works.

## Run a simulation

For example, localization with an Extended Kalman Filter, where the vehicle estimates its position from sensor observations while following a course:

```bash
$ python src/simulations/localization/extended_kalman_filter_localization/extended_kalman_filter_localization.py
```

A window opens and the animation plays. The terminal prints `Simulation start!!` when it begins and `Simulation finished!!` when it has played through once (it does not loop); close the window to end the process.

To pick a simulation, browse the [gallery of simulations](/doc/EXAMPLES.md): it lists every simulation with its demo image, and each one runs the same way.

The window itself — the world it shows, the time steps it advances by, and the coordinate frames — is the subject of [Chapter 2](/doc/simulator/index.md).

## Save the animation as a GIF

Every script accepts a `--save-gif` command line option. About half of the simulations define a GIF output file (most of the path planners, and controllers such as MPPI and MPC). For them, the option writes the GIF file into the simulation's own folder, next to its script, instead of opening the window:

```bash
$ python src/simulations/path_planning/astar_hybrid_path_planning/astar_hybrid_path_planning.py --save-gif
```

This writes `astar_hybrid_search.gif` and `astar_hybrid_navigate.gif` into `src/simulations/path_planning/astar_hybrid_path_planning/`. These GIF files are part of the repository — the gallery shows them — so running with `--save-gif` regenerates them.

A simulation without a GIF output file ignores the option and shows the window as usual.

## Run the tests

Each script keeps its animation behind a `show_plot` flag at the top of the file (`show_plot = True`). The test suite imports every script and calls its `main()` with the flag off, so the unit tests run the same computation as the animation, windowless. Running them is the quickest way to confirm that your environment is complete:

```bash
$ . run_test_suites.sh
```

This runs pytest over the whole `test/` directory (the script is `pytest -l --durations=0`), which takes a few minutes. A final line like `110 passed` means every simulation ran through its computation without errors, and your setup is ready.
