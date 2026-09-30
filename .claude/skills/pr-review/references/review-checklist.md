# Review Checklist

Points to check in a PR to this repository. Add new points here as they come up in reviews.

## Conventions

- Python only, using only libraries listed in the Requirements section of `README.md`
- Modules are loaded with `sys.path.append(...)` like existing files, not as packages
- Simulation scripts have the docstring template in `generate_example_gallery.py` (file name, `Title:`, `Description:`, `Author:`, nothing else). Other files have the `file name / Author` docstring
- Class and constructor docstrings state the meaning and unit of each argument, and variable names include units (`_m`, `_rad`, `_mps`)
- Simulation scripts have a `show_plot` flag and keep the logic in `main()`
- A new simulation has `test/test_<simulation_name>.py`, and new components have unit tests for their logic
- `pyrightconfig.json` and `.devcontainer/devcontainer.json` are regenerated when new module directories are added
- Comments, docstrings, messages and docs are in English

## Learning material

- The algorithm is implemented as simply as possible and is easy to follow, not optimized for practical use
- Pure logic (calculation) is separated from drawing so it can be read and tested on its own
- Formulas match the referenced paper, and any simplification (e.g. a fixed parameter) is stated in the docstring
- The visualization doesn't mislead readers (e.g. normalization that differs from what the logic uses)

## Integration

- Existing components (vehicle, sensors, mapper/controller slots) are reused instead of being copied or changed
- Coordinate frames and origins are consistent with the components the data comes from (vehicle origin vs sensor position, vehicle vs global frame)

## Performance

- The simulation runtime is close to similar existing simulations. Watch for per-element matplotlib calls in `draw()` (e.g. `add_patch` per patch) that should be batched into a collection

## Demo and gallery

- A demo GIF is added under the simulation directory, shows the algorithm working, and is not much larger than existing ones (a few MB)
- The `Title` reads well as a gallery heading and is consistent in style with existing titles
