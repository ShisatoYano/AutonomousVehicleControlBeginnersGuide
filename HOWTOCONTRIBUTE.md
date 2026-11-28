# How to contribute
This document describes how to contribute this project.  
Before starting your contribution, please check [README](/README.md) to understand what this project is and setup your development environment.  
If you use an AI coding tool such as Claude Code, the project's conventions are summarized in [CLAUDE.md](/CLAUDE.md), which Claude Code loads automatically.  
For questions and ideas, please use [Discussions](https://github.com/ShisatoYano/AutonomousVehicleControlBeginnersGuide/discussions). Issues are for bug reports, proposals and agreed tasks.  

## Supported Python versions

The project supports every Python version that has not reached its end of life,
so learners can use the interpreter that ships with their OS. The Linux, macOS
and Windows CI workflows run the test suite on every supported version.

| Role | Version | Used by |
|---|---|---|
| Newest supported | 3.15 | CI matrix |
| Recommended | 3.14 | README, Dockerfile, dev container, book and gallery workflows |
| Oldest supported | 3.11 | CI matrix |

The recommended version is the newest release minus one. A new Python version is
not recommended right after its release because the dependencies (NumPy, SciPy,
matplotlib, CasADi, do-mpc) may not have wheels for it yet. The Dockerfile names
the minor version without a patch number (`python:3.14`), so rebuilding the image
also picks up patch and security fixes.

"Supported" means the code runs and the test suite passes on that version. It
does not mean every dependency publishes wheels for it: pip installs the newest
release of each dependency that still supports the interpreter, so Python 3.11
gets older NumPy and SciPy releases than Python 3.15.

### Yearly review

Review the policy every year in December or January, after the October Python
release:

1. Add the new version to the CI matrix once its dependencies publish wheels.
   If one of them does not, leave the version out and add a comment in the
   workflow saying which dependency blocks it.
2. Move the recommended version up by one, and update the files in the table.
3. Remove versions that reached their end of life from the matrix and the table.

## Claim an issue before you start
To avoid two people working on the same thing, please claim an issue before you start working on it.  

1. Anyone can claim an issue labeled `good first issue` or `help wanted`: comment `/claim` on the issue (on the first line of the comment), and the issue is assigned to you automatically. A reply tells you if it can't be, for example because someone else is already assigned or the issue is labeled `multiple PRs` (claim one of its child issues instead). Other issues, such as a proposal or a defect report, are assigned by the maintainer after the maintainer agrees with them, even if you opened the issue yourself. If you want to work on such an issue, say so in it.  
2. If neither a pull request nor a comment from you on the issue appears within 14 days after you were assigned, the maintainer unassigns you so that someone else can take the issue. You can claim it again later.  
3. You can have only one open `good first issue` assigned to you at a time, so that other newcomers can also find one.  
4. Open a pull request only after you were assigned to its issue. Write "Closes #123" in the description, or "Part of #123" if the issue is labeled `multiple PRs` (see Step 4). The `PR_Evidence` check fails if none of the issues in the description is assigned to you; after you are assigned, edit the description to run it again.  

Small fixes such as a typo don't need an issue. Open a pull request directly, and the maintainer adds the `no issue needed` label.  

## Response times
The maintainer promises the following.  

* The first reply to a new pull request within 48 hours.  
* At least 8 open `good first issue` issues that nobody is assigned to, so that a newcomer can always find one.  

## Releases
The project is released at the beginning of every month, raising the minor version (v0.1.0, v0.2.0, ...). A pull request merged during a month is included in the next month's release. Only an urgent fix is released in between, raising the patch version (for example v0.1.1).  

The release notes are generated from the merged pull requests and grouped by their labels: `breaking change`, `documentation` (shown as "Book"), `enhancement` and `bug`. The maintainer adds one of these labels to each pull request, so the release notes show it in the right group. The version of the book is shown at the bottom of each page.  

## Add new sample program of algorithm
Steps to add a new sample program.  

### Step 1: Propose by creating issue
If you found an algorithm which you want to implement, let's create an issue to propose it on GitHub. We will check it and give a comment as soon as possible.  

If we agreed with your proposal, let's go to Step 2.  
If you want to implement it yourself, say so in the issue. Once we agree with the proposal, the maintainer assigns the issue to you, as described in [Claim an issue before you start](#claim-an-issue-before-you-start). Please start implementing after that.  

It is OK to just create an issue to propose. Someone might see your proposal and implement in the future. In this case, please share a paper or documentation about the algorithm to understand it.  

### Step 2: Implement sample program of algorithm
When you implement a sample program of an algorithm, please keep the following items in mind.  

1. Use only Python. Using other language is not acceptable.  
2. Acceptable version of Python is 3.x.  
3. Implement an animation by using matplotlib to show how the algorithm works.  
4. Use only the libraries listed in [requirements.txt](/requirements.txt).  
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
* Images or GIFs that show the result, so the reviewer can see it without running the code. They are needed for any new or changed code under `src/`: the demo GIF of a new simulation or algorithm, and for a change to existing code (a bug fix, an improvement or a refactoring), the last frame of one or two simulations that use it, before and after your change. Drag and drop them into the description.  
* If your change touches code that the [book](https://shisatoyano.github.io/AutonomousVehicleControlBeginnersGuide/) describes, the updated book pages in the same PR.  

If your PR does one part of an issue labeled `multiple PRs` (for example one section of a book chapter, or one category of simulations), write "Part of #123" in the description instead of "Closes #123", so the issue stays open for the other parts.  

"How I checked it works" and, for a change to code under `src/`, the images are required: the `PR_Evidence` check fails without them, and the review starts once they are there. Editing the description runs the check again. If a change under `src/` has nothing to show (for example a docstring only), say why under "Images", and the maintainer adds the `no images needed` label.  

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

Additionally, you can see existing reported issues and fix the defect. Before you start fixing, get assigned to the issue as described in [Claim an issue before you start](#claim-an-issue-before-you-start). And then, please don't forget to add a unit test code to check the defect was fixed. After you completed fixing, let's submit a PR following the rule of Step 4.  

## Add documentation about existing program
Adding a documentation about existing programs is also welcome.  
To write or improve a chapter of the book, follow the [Writing Guide](https://shisatoyano.github.io/AutonomousVehicleControlBeginnersGuide/WRITING_GUIDE.html) and the [Notation and Units](https://shisatoyano.github.io/AutonomousVehicleControlBeginnersGuide/NOTATION.html) page (sources: [doc/WRITING_GUIDE.md](/doc/WRITING_GUIDE.md), [doc/NOTATION.md](/doc/NOTATION.md)). If you have a suggestion about how to write the documents, please submit a PR and I will review it.  
A small fix to an existing section of the book, such as a typo or wording, doesn't need an issue: open a PR directly. To write a new section or largely extend one, open an issue with the [Book section claim](https://github.com/ShisatoYano/AutonomousVehicleControlBeginnersGuide/issues/new?template=book_section_claim.yml) template and start writing after you are assigned to it.  

The documents under `doc/` are published as a [web book](https://shisatoyano.github.io/AutonomousVehicleControlBeginnersGuide/) with Sphinx and MyST, so they stay in Markdown. To check your changes, build the book locally and open `doc/_build/html/index.html` in a browser:

```bash
pip install -r doc/requirements.txt
sphinx-build -W --keep-going -b html doc doc/_build/html
```

`-W` turns warnings such as broken links into errors, as in GitHub Actions. A pull request that changes `doc/` builds the book there, and the book is published again after the merge.  

## Support this project
Supporting this project financially is also a great contribution!!.  
If you or your company could support this project, please consider [Sponsor @ShisatoYano on GitHub Sponsors](https://github.com/sponsors/ShisatoYano).  
