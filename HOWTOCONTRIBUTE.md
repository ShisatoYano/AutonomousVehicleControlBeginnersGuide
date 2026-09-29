# How to contribute
This document describes how to contribute this project.  
Before starting your contribution, please check [README](/README.md) to understand what this project is and setup your development environment.  
If you use an AI coding tool such as Claude Code, the project's conventions are summarized in [CLAUDE.md](/CLAUDE.md), which Claude Code loads automatically.  

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

### Step 3: Implement unit test
If you added a new sample program, please add a unit test code for it under test directory. When you implement the test code, please refer to the existing code. Additionally, each unit tests should be able to run without animation because test process stops and an exception occurs. After you completed to implement the test code, confirm the test passes by executing the script, run_test_suites.sh/bat.  

If your sample program has a demo GIF/image under its `src/simulations/...` directory, it is added to [doc/EXAMPLES.md](/doc/EXAMPLES.md) automatically after your PR is merged, so you don't need to update that file. Please don't edit `doc/EXAMPLES.md` by hand, because it is overwritten. To change the heading or description, edit the docstring of your simulation script (see Step 2).  

### Step 4: Submit a pull request and modify code based on review
If your sample program and test were ready, let's create a pull request and submit it. When you create the PR, please write a description about the following items.  

* The overview of your PR.  
* How did you confirm that your program works well.  

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
There have not been any rules about documentation yet. If you had any suggestion of how to write documentations, please submit a PR and I will review it.  

## Support this project
Supporting this project financially is also a great contribution!!.  
If you or your company could support this project, please consider [Sponsor @ShisatoYano on GitHub Sponsors](https://github.com/sponsors/ShisatoYano).  
