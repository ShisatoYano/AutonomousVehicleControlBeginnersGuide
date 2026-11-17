# 1.1 Setting Up Python and the Dependencies

How to set up the Python environment for the autonomous driving simulations: choose between a local Python installation and a Docker container, install the libraries, and check that a simulation runs.

The commands for each way are in the [Quick Start of the README](/README.md#2-set-up-the-environment). This section explains which way to choose, what each step does, and what to do when something goes wrong. After it, you have an environment that runs every simulation in the repository.

## What you need

- **Git**, to get the repository: `git clone https://github.com/ShisatoYano/AutonomousVehicleControlBeginnersGuide`
- **Python**, in the version listed under [Requirements in the README](/README.md#requirements)
- **The libraries in `requirements.txt`**:

| Library | Used for |
|---|---|
| NumPy, SciPy | The computations of every algorithm |
| Matplotlib | Drawing the animations |
| CasADi, do-mpc | Only the model predictive control (MPC) simulation |
| pytest | Running the tests |

## Choose a way to set up

The README describes three ways. They give the same result; they differ in what you install on your computer, and in how the animation window reaches your screen.

| | A. VS Code + Dev Container | B. Docker only | C. Local Python |
|---|---|---|---|
| You install | Docker, VS Code and its Dev Containers extension | Docker | Python |
| Python version | Set by the container | Set by the container | The one you install |
| Animation window | Through an X server (see below) | Through an X server | Opens directly |
| Good when | You want an editor and an environment that are set up for you | You don't use VS Code, or work on a remote machine | You want the shortest setup, on any operating system |

If you are not sure, start with **C**. It works the same way on Linux, macOS and Windows, and the windows open without extra setup. The automatic tests of the repository (GitHub Actions) also run on a local Python, without a container, on all three operating systems.

Choose **A** or **B** if you don't want to change the Python on your computer: everything is installed inside the container, and removing the container and its image removes it.

## What each step does

### C. Local Python: a virtual environment

```bash
$ python3 -m venv .venv
$ source .venv/bin/activate
$ pip install -r requirements.txt
```

The first command creates a **virtual environment** in the folder `.venv`: a private copy of Python with its own libraries. The second activates it in your shell (the prompt starts with `(.venv)`), so `python` and `pip` now mean the ones in `.venv`. The third installs the libraries into it. The libraries of your other projects stay untouched, and deleting `.venv` removes everything.

Activate the environment again in every new shell before running a simulation. On Windows, the commands differ slightly; they are in the README.

### A and B. Docker: a container with everything installed

The `Dockerfile` describes the container: a Linux system with Python, and the libraries of `requirements.txt` installed. `docker compose up -d --build` builds it the first time and starts it. Option A does the same from VS Code and also installs the editor extensions.

Two things to know:

- **The repository folder is shared.** The container sees your copy of the repository (it is mounted at `/home/dev-user/workspace`), so you edit the files with any editor on your computer, and the container runs your latest version.
- **The animation window comes through X11.** The container has no screen of its own: it draws the window on the display of your computer through the X Window System (X11). On Linux, allow the container to use your display with `xhost +local:`. On Windows and macOS, you need an X server first: WSLg on WSL2, or XQuartz on macOS.

## Check the setup

Run one simulation from the repository root:

```bash
$ python src/simulations/path_tracking/pure_pursuit_path_tracking/pure_pursuit_path_tracking.py
```

A window opens, and a vehicle follows a course. The terminal prints `Simulation start!!` and `Simulation finished!!`. If you see this, your environment is ready. [Section 1.2](/doc/getting_started/running_simulations.md) explains how to run the other simulations, save animations and run the tests.

## When something goes wrong

- **`ModuleNotFoundError: No module named 'numpy'`** (or another library): the libraries are not in the Python you are running. With option C, activate the virtual environment first, and check that the prompt starts with `(.venv)`.
- **Libraries of other projects appear, or the wrong version is imported:** another project may have set the environment variable `PYTHONPATH` (for example, the setup script of a ROS workspace). It adds folders to every Python, including the virtual environment. Run the simulations in a shell where `echo $PYTHONPATH` prints nothing, or run `unset PYTHONPATH` first.
- **`pip install` fails while building a library:** your Python may be newer than the libraries support yet, or older than the code needs. Check `python --version` against the version in the README.
- **No window appears in the container, or an error mentions `DISPLAY`:** the container can't reach your display. Run `xhost +local:` again on Linux (the setting is reset when you log out), or start your X server on Windows and macOS.
- **Warnings when the MPC simulation starts** (for example "The ONNX feature is not available"): do-mpc reports optional features that the simulation doesn't use. They are harmless.
