"""
conftest.py

Test configuration for deterministic test runs

Selects a non-interactive matplotlib backend and reseeds the random number
generator for each test, so repeated test runs give the same results on any
machine and no component's global state leaks between tests.

Author: Goldloli
GitHub: https://github.com/Goldloli
"""

import matplotlib

matplotlib.use("Agg")

import numpy as np
import pytest


@pytest.fixture(autouse=True)
def reseed_random_numbers_for_each_test():
    np.random.seed(42)
    yield
