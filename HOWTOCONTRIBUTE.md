# How to contribute
This document describes how to contribute this project.  
Before starting your contribution, please check [README](/README.md) to understand what this project is and setup your development environment.  
If you use an AI coding tool such as Claude Code, the project's conventions are summarized in [CLAUDE.md](/CLAUDE.md), which Claude Code loads automatically.  
For questions and ideas, please use [Discussions](https://github.com/ShisatoYano/AutonomousVehicleControlBeginnersGuide/discussions). Issues are for bug reports and agreed tasks.  

## Add new sample program of algorithm
Steps to add a new sample program.  

### Step 1: Propose by creating issue
If you found an algorithm which you want to implement, let's create an issue to propose it on GitHub. We will check it and give a comment as soon as possible.  

If we agreed with your proposal, let's go to Step 2.  

It is OK to just create an issue to propose. Someone might see your proposal and implement in the future. In this case, please share a paper or documentation about the algorithm to understand it.  

### Step 2: Implement sample program of algorithm
When you implement a sample program of an algorithm, please keep the following items in mind.  

1. Use only Python. Using other language is not acceptable.  
2. Acceptable version of Python is 3.x.  
3. Implement an animation by using matplotlib to show how the algorithm works.  
4. Use only libraries and tools written in Requirements section of README.  
5. Implement as simple as possible. The main purpose to help a user to understand the algorithm. It is not for practical usage.  
6. Put the following docstring at the top of your simulation script under `src/simulations/`. `Title` is used as the heading of [doc/EXAMPLES.md](/doc/EXAMPLES.md), and `Description` and `Author` are shown under it. All three are required, each on one line, and nothing else can be written in the docstring (the unit test fails otherwise). If your simulation directory has multiple scripts, use the same `Title` in all of them.  

```python
"""
pure_pursuit_path_tracking.py

Title: Pure Pursuit Path Tracking
Description: One-line summary shown under the heading
Author: Your Name
"""
```

7. Add type hints to the arguments and return values of functions and methods. Keep them simple (`list[float]`, `float | None`), write NumPy arrays as `npt.NDArray[np.float64]`, and write their shapes in the docstring. Existing code without type hints doesn't need to be changed in your PR.  

#### Code completion and import resolution in your editor
Each simulation script adds its own module search paths at runtime with
`sys.path.append(...)`, since this project doesn't use a standard Python
package layout. Static analysis tools (Pylance/pyright) can't see those
runtime `sys.path.append(...)` calls, so without extra configuration your
editor will show false "import could not be resolved" errors and code
completion won't work for these modules.

To fix this, `pyrightconfig.json` (for pyright/Pylance, e.g. Neovim or VS
Code without Dev Containers) and `.devcontainer/devcontainer.json`'s
`python.analysis.extraPaths` (for VS Code + Dev Containers) list every
directory under `src/components` and `src/simulations` that directly
contains a `.py` file.

Whenever you add a new module (a new directory under `src/components` or
`src/simulations`), regenerate both files by running:
```bash
$ python generate_pyrightconfig.py
```
This scans the project and rewrites both files automatically, so you don't
need to edit them by hand.

Run this script on your host machine, not inside the container. It only
uses the Python standard library (`json`, `pathlib`), and since the project
directory is bind-mounted into the container as-is, the files it writes are
the same on both sides either way. Running it on the host is simplest since
it doesn't depend on the container's Python environment at all.

### Step 3: Implement unit test
If you added a new sample program, please add a unit test code for it under test directory. When you implement the test code, please refer to the existing code. Additionally, each unit tests should be able to run without animation because test process stops and an exception occurs. After you completed to implement the test code, confirm the test passes by executing the script, run_test_suites.sh/bat.  

If your sample program has a demo GIF/image under its `src/simulations/...` directory, it is added to [doc/EXAMPLES.md](/doc/EXAMPLES.md) automatically after your PR is merged, so you don't need to update that file. Please don't edit `doc/EXAMPLES.md` by hand, because it is overwritten. To change the heading or description, edit the docstring of your simulation script (see Step 2).  

Also add your simulation to the chapter of the [book](https://shisatoyano.github.io/AutonomousVehicleControlBeginnersGuide/) it belongs to: add a link to its folder in the "Sections" list of `doc/<chapter>/index.md`, for example ``[`ndt_map_construction`](/src/simulations/mapping/ndt_map_construction)`` in `doc/mapping/index.md`. The test `test/test_book_chapters.py` checks that every simulation is in exactly one chapter. If you are not sure which chapter fits, ask in your issue or pull request.  

Run a simulation normally to display its animation in a window. To regenerate a committed demo GIF, run the simulation with the `--save-gif` option, for example:

```bash
python src/simulations/path_tracking/mppi_path_tracking/mppi_path_tracking.py --save-gif
```

The option saves the GIF at the path defined by the simulation. Without it, existing GIF paths are not overwritten and the animation is displayed instead.

### Step 4: Submit a pull request and modify code based on review
If your sample program and test were ready, let's create a pull request and submit it. The description starts with a template; please fill in the following items.  

* The overview of your PR.  
* How did you confirm that your program works well.  
* Images or GIFs that show the result, so the reviewer can see it without running the code: the demo GIF of a new simulation, or for a change to code under `src/components/`, the last frame of one or two simulations that use it, before and after your change. Drag and drop them into the description.  

After you submitted your PR, each unit tests is executed automatically by GitHub Actions. If an error occurred and unit tests failed, please investigate the reason and fix it. After all tests passed and any problems were not found by code review, I will allow you to merge your PR into main branch.  

## Report and fix defect
Reporting and fixing a defect are also welcome.  
When you report an issue, please provide the following information.  

* A clear and concise description about the defect.  
* A clear and concise description about your expectation.  
* Screenshots to help explaining the defect.  
* OS version.  
* Python version.  
* Each libraries version.  

Additionally, you can see existing reported issues and fix the defect. And then, please don't forget to add a unit test code to check the defect was fixed. After you completed fixing, let's submit a PR following the rule of Step 4.  

## Add documentation about existing program
Adding a documentation about existing programs is also welcome.  
To write or improve a chapter of the book, follow the [Writing Guide](https://shisatoyano.github.io/AutonomousVehicleControlBeginnersGuide/WRITING_GUIDE.html) and the [Notation and Units](https://shisatoyano.github.io/AutonomousVehicleControlBeginnersGuide/NOTATION.html) page (sources: [doc/WRITING_GUIDE.md](/doc/WRITING_GUIDE.md), [doc/NOTATION.md](/doc/NOTATION.md)). If you have a suggestion about how to write the documents, please submit a PR and I will review it.  

The documents under `doc/` are published as a [web book](https://shisatoyano.github.io/AutonomousVehicleControlBeginnersGuide/) with Sphinx and MyST, so they stay in Markdown. To check your changes, build the book locally and open `doc/_build/html/index.html` in a browser:

```bash
pip install -r doc/requirements.txt
sphinx-build -W --keep-going -b html doc doc/_build/html
```

`-W` turns warnings such as broken links into errors, as in GitHub Actions. A pull request that changes `doc/` builds the book there, and the book is published again after the merge.  

## Support this project
Supporting this project financially is also a great contribution!!.  
If you or your company could support this project, please consider [Sponsor @ShisatoYano on GitHub Sponsors](https://github.com/sponsors/ShisatoYano).  
