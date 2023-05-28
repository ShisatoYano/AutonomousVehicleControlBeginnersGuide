"""
Unit test of ObstacleList

Author: Shisato Yano
"""

import matplotlib as mpl
mpl.use("Agg")
import matplotlib.pyplot as plt
import pytest
import sys
from pathlib import Path

sys.path.append(str(Path(__file__).absolute().parent) + "/../src/obstacle")
from obstacle_list import ObstacleList


class MockObstacle:
    def __init__(self):
        pass

    def update(self, time_s):
        pass

    def draw(self, axes, elems):
        pass


def test_initialize():
    obsts = ObstacleList()
    assert len(obsts.list) == 0


def test_add_obstacle():
    obsts = ObstacleList()

    obst1 = MockObstacle()
    obsts.add_obstacle(obst1)
    assert len(obsts.list) == 1

    obst2 = MockObstacle()
    obsts.add_obstacle(obst2)
    assert len(obsts.list) == 2


def test_update():
    obsts = ObstacleList()

    obst1 = MockObstacle()
    obsts.add_obstacle(obst1)
    obsts.update(1.0)


def test_draw():
    obsts = ObstacleList()

    obst1 = MockObstacle()
    obsts.add_obstacle(obst1)

    plt.clf()
    plt.close()

    figure = plt.figure(figsize=(8, 8))
    axes = figure.add_subplot(111)

    obsts.draw(axes, [])


def test_get_list():
    obsts = ObstacleList()

    obst1 = MockObstacle()
    obsts.add_obstacle(obst1)

    assert len(obsts.get_list()) == 1
