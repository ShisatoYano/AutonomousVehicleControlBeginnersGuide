# How to contribute
This document describes how to contribute this project.  
Before starting your contribution, please check [README](/README.md) to understand what this project is and setup your development environment.  

## Add new sample program of algorithm
Steps to add a new sample program.  

### Step 1: Propose by creating issue
If you found an algorithm which you want to implement, let's create an issue to propose it on GitHub. We will check it and give a comment as soon as possible.  

If we agreed with your proposal, let's go to Step 2.  

It if OK to just create an issue to propose. Someone might see your proposal and implement in the future. In this case, please a paper or ducumentation about the algorithm to understand it.  

### Step 2: Implement sample program of algorithm
When you implement a sample program of an algorithm, please keep the following items in mind.  

1. Use only Python. Using other language is not acceptable.  
2. Acceptable version of Python is 3.x.  
3. Implement an animation by using matplotlib to show how the algorithm works.  
4. Use only libraries and tools written in Requirements section of README.  
5. Implement as simple as possible. The main purpose to help a user to understand the algorithm. It is not for practical usage.  

### Step 3: Implement unit test
If you added a new sample program, please add a unit test code for it under test directory. When you implement the test code, please refer to the existing code. Additionally, each unit tests should be able to run without animation because test process stops and an exception occurs. After you completed to implement the test code, confirm the test passes by executing the script, run_test_suites.sh/bat.  

### Step 4: Submit a pull request and modify code based on review

## Report and fix defect

## Add documentation about existing program

## Support this project