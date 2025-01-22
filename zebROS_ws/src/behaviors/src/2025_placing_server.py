#!/usr/bin/env python3

import rospy
import actionlib

from behavior_actions.msg import Placing2025Goal, Placing2025Feedback, Placing2025Result, Placig2025Action
from behavior_actions.msg import Elevater2025Goal, Elevater2025Feedback, Elevater2025Result, Elevater2025Action
from behavior_actions.msg import Roller2025Goal, Roller2025Feedback, Roller2025Result, Roller2025Action

class PlacingServer(object):
    def __init__(self, name):
        self.__action_name = name
        self.result = Placing2025Result()
        self.feedback = Placing2025Feedback()
        self.elevater_client = actionlib.SimpleActionClient('Elevater', Elevater2025Action)
        rospy.loginfo('Waiting for Elevater action server')
        self.elevater_client.wait_for_server()
        self.roller_client = actionlib.SimpleActionClient('Roller', Roller2025Action)
        rospy.loginfo('Waiting for Roller action server')
        self.roller_client.wait_for_server()

    def execute_cb(self, goal):
        pass