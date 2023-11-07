#!/usr/bin/env bash

echo "Run test suites!! "

# === pytest based test runner ===
# -l (--showlocals); show local variables when test failed
# --durations=0: show ranking of test durations
pytest -l --durations=0