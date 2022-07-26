@echo off

echo Run test suites!!

rem === pytest based test runner ===
rem -l (--showlocals); show local variables when test failed
rem -Werror: warning as error
rem --durations=0: show ranking of test durations
rem --cov -v: show test coverage
pytest -l -Werror --durations=0 --cov -v