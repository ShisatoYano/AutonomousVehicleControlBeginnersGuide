"""
Unit test of StateHistory

Author: Shisato Yano
"""

import pytest
import sys
import matplotlib.pyplot as plt
from pathlib import Path

sys.path.append(str(Path(__file__).absolute().parent) + "/../src/state")
from state_history import StateHistory


# mock class
class MockSpecification:
    def __init__(self):
        self.color = 'k'


# test instance
spec = MockSpecification()
history = StateHistory([0.0], [1.0], spec.color)


def test_initialization():
    assert len(history.x_history) == 1
    assert len(history.y_history) == 1

    assert history.x_history[0] == 0.0
    assert history.y_history[0] == 1.0


def test_update():
    updated_history = history.update(1.0, 0.0)

    assert len(updated_history.x_history) == 2
    assert len(updated_history.y_history) == 2

    assert updated_history.x_history[0] == 0.0
    assert updated_history.y_history[0] == 1.0
    assert updated_history.x_history[1] == 1.0
    assert updated_history.y_history[1] == 0.0


def test_draw():
    figure = plt.figure(figsize=(8, 8))
    axes = figure.add_subplot(111)
    history.draw(axes, [])

