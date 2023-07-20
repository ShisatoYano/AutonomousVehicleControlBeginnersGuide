#!/usr/bin/env bash

echo "Run test suites!! "

# === pytest based test runner ===
# -l (--showlocals); show local variables when test failed
# -Werror: warning as error
# --durations=0: show ranking of test durations
pytest -l -Werror --durations=0