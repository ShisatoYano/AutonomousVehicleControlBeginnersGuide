# Testing standards
This document is about standards of Unit test.  
Each sample programs are modularized and can be executed independently. Some of programs are implemented by importing multiple modules. When you create a pull request, please confirm all of programs can be executed correctly by Unite test in advance.  

## How to create test code
All of test codes are located at "Test" directory. And then, a name of test code should be "test_xxx.py". For example, when the name of test target file is "hoge.py", you need to create "test_hoge.py" as test code file.  

## Framework
Each test code is implemented by using Python's testing framework, [pytest](https://github.com/pytest-dev/pytest).  

## How to run test
You can run all of unit tests by executing the following command.  
On Linux,    
```bash
$ ./run_test_suites.sh
```
On Windows,  
```bash
.\run_test_suites.bat
```
If you want to run certain unit test, you can do by executing the following command.  
```bash
$ pytest test/test_hoge.py
```

## Automated test by GitHub Actions
When you created a pull request or your code was merged into main branch、all of unit tests are also executed by [GitHub Actions](https://github.com/ShisatoYano/AutonomousDrivingSamplePrograms/actions).  

