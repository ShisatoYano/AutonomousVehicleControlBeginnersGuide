"""
agent.py

Author: Shisato Yano
"""

class Agent:
    """
    Agent class
    """

    def __init__(self, speed_mps, yaw_rate_rps):
        self.speed_mps = speed_mps # m/s
        self.yaw_rate_rps = yaw_rate_rps # rad/s

    def control_input(self):
        return self.speed_mps, self.yaw_rate_rps
