"""
Unit test of Agent

Author: Shisato Yano
"""

import pytest
import sys
import os

sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../src/agent")
from agent import Agent


# test instance
agent = Agent(1.0, 2.0)


def test_create_instance():
    assert agent.speed_mps == 1.0
    assert agent.yaw_rate_rps == 2.0


def test_control_order():
    assert agent.control_order()[0, 0] == 1.0
    assert agent.control_order()[1, 0] == 2.0
