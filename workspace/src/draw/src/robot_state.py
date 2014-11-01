"""
Maintains a global "Robot" variable that
subscribes to various topics and records their values
"""

import rospy
import numpy as np

class RobotState(object):
    def __init__(self):
        self.__subscribe()

    def __subscribe(self):
        """
        Initialize all subscribers
        """
        self.pic2world_sub = rospy.Subscriber('renoir/pic2world', lambda msg: self.__pic2world_callback(self))

    def __publish(self):
        """
        Initialize all publishers. Empty for now
        """
        pass

    def __pic2world_callback(self, msg):
        self.pic2world_transform = msg

    def getPic2World(self):
        """
        Return the latest picture to world frame transformation matrix.
        This is a 3x2 matrix which brings picture frame coordinates (2D) to world
        frame coordinates in Baxter's base frame
        """
        return self.pic2world_transform

ROBOT_STATE = RobotState()
