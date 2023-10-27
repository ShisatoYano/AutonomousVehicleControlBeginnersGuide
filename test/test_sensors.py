"""
Unit test of OmniDirectionalLidar

Author: Shisato Yano
"""

import matplotlib as mpl
mpl.use("Agg")
import matplotlib.pyplot as plt
import pytest
import sys
from pathlib import Path

sys.path.append(str(Path(__file__).absolute().parent) + "/../src/components/sensors")
from sensors import Sensors


# mock classes
class MockLidar:
    def __init__(self):
        self.point_cloud = [None]

    def install(self, state):
        pass

    def update(self, state):
        pass

    def draw(self, axes, elems, state):
        pass

    def get_point_cloud(self):
        return self.point_cloud


class MockGnss:
    def __init__(self):
        pass
    
    def install(self, state):
        pass

    def update(self, state):
        pass

    def draw(self, axes, elems):
        pass


class MockState:
    def __init__(self):
        pass


# test instance
lidar = MockLidar()
gnss = MockGnss()
sensors = Sensors(lidar, gnss)
state = MockState()


def test_initialize():
    sensors_none = Sensors()
    assert sensors_none.lidar == None
    assert sensors_none.gnss == None

    assert sensors.lidar != None
    assert sensors.gnss != None


def test_install():
    sensors.install(state)


def test_update_data():
    sensors.update_data(state)


def test_draw():
    plt.clf()
    plt.close()
    
    figure = plt.figure(figsize=(8, 8))
    axes = figure.add_subplot(111)

    sensors.draw_data(axes, [], state)


def test_get_point_cloud_from_lidar():
    sensors_no_lidar = Sensors(gnss=gnss)
    assert len(sensors_no_lidar.get_point_cloud_from_lidar()) == 0

    assert len(sensors.get_point_cloud_from_lidar()) == 1
