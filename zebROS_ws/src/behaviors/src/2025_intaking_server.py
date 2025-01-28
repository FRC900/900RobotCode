#!/usr/bin/env python3

import rospy
import actionlib

from behavior_actions.msg import Intaking2025Goal, Intaking2025Feedback, Intaking2025Result, Intaking2025Action
from behavior_actions.msg import Elevater2025Goal, Elevater2025Feedback, Elevater2025Result, Elevater2025Action
from behavior_actions.msg import Roller2025Goal, Roller2025Feedback, Roller2025Result, Roller2025Action
from behavior_actions.msg import Intaker2025Goal, Intaker2025Feedback, Intaker2025Result, Intaker2025Action

class IntakingServer(object):
    def __init__(self, name):
        self.__action_name = name
        self.result = Intaking2025Result()
        self.goal = Intaking2025Goal()
        self.feedback = Intaking2025Feedback()
        self.elevater_client = actionlib.SimpleActionClient('Elevater', Elevater2025Action)
        rospy.loginfo('Waiting for Elevater action server')
        self.elevater_client.wait_for_server()
        self.roller_client = actionlib.SimpleActionClient('Roller', Roller2025Action)
        rospy.loginfo('Waiting for Roller action server')
        self.roller_client.wait_for_server()
        self.intaker_client = actionlib.SimpleActionClient('Intaker', Intaker2025Action)
        rospy.loginfo('Waiting for Intaker action server')
        self.intaker_client.wait_for_server()


    def execute_cb(self, goal : Intaking2025Goal):
        elevater_result = Elevater2025Result()
        self.feedback.state = self.feedback.NOT_ACQUIRED
        self.server.publish_feedback(self.feedback)
        if not elevater_result.success:
            elevater_goal = Elevater2025Goal()
            elevater_goal.mode = 4
            def elevater_done_cb(state, result):
                rospy.loginfo('Elevater action finished')
            self.elevater_client.send_goal(elevater_goal.mode, elevater_done_cb)
        rospy.loginfo('Elevater action passed check, now intaking')
        intaker_goal = Intaker2025Goal()
        def intaker_done_cb(state, result):
            rospy.loginfo('Intaker action finished')
        self.intaker_client.send_goal(intaker_goal)
        rospy.loginfo('Intaker action sent')
        self.feedback.state = self.feedback.IN_INTAKE
        self.server.publish_feedback(self.feedback)
        roller_goal = Roller2025Goal()
        roller_goal.mode = 0 # Set the roller goal to INTAKE
        def roller_done_cb(state, result):
            rospy.loginfo('Roller action finished')
        self.roller_client.send_goal(roller_goal.mode, roller_done_cb)
        rospy.loginfo('Roller action sent')
        self.feedback.state = self.feedback.IN_ROLLER
        self.server.publish_feedback(self.feedback)
        self.result.success = True
        rospy.loginfo("2025_intaking_server: succeeded")
        self.server.set_succeeded(self.result) 


if __name__ == '__main__':
    rospy.init_node('2025_intaking_server')
    server = IntakingServer(rospy.get_name())
    rospy.spin()