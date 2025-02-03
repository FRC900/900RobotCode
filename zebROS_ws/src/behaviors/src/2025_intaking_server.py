#!/usr/bin/env python3

import rospy
import actionlib

from behavior_actions.msg import Intaking2025Goal, Intaking2025Feedback, Intaking2025Result, Intaking2025Action
from behavior_actions.msg import Elevater2025Goal, Elevater2025Feedback, Elevater2025Result, Elevater2025Action
from behavior_actions.msg import Roller2025Goal, Roller2025Feedback, Roller2025Result, Roller2025Action
from behavior_actions.msg import Intaker2025Goal, Intaker2025Feedback, Intaker2025Result, Intaker2025Action
from talon_controller_msgs.srv import Command, CommandRequest


class IntakingServer(object):
    def __init__(self, name):
        self.result = Intaking2025Result()
        self.goal = Intaking2025Goal()
        self.feedback = Intaking2025Feedback()
        self.elevater_client = actionlib.SimpleActionClient('/elevater/elevater_server_2025', Elevater2025Action)
        rospy.loginfo('Waiting for Elevater action server')
        self.elevater_client.wait_for_server()
        self.roller_client = actionlib.SimpleActionClient('/roller/roller_server_2025', Roller2025Action)
        rospy.loginfo("Found elevater server")

        self.intake_client = rospy.ServiceProxy(f"/frcrobot_jetson/{rospy.get_param('controller_name')}/command", Command)
        rospy.loginfo(f"Waiting for intaking service at /frcrobot_jetson/{rospy.get_param('controller_name')}/command")
        self.intake_client.wait_for_service()

        self.server = actionlib.SimpleActionServer(name, Intaking2025Action, execute_cb=self.execute_cb, auto_start = False)
        self.server.start()
        rospy.loginfo("Intaking actionlib server")

    def execute_cb(self, goal : Intaking2025Goal):
        self.feedback.state = self.feedback.NOT_ACQUIRED
        self.server.publish_feedback(self.feedback)

        elevater_goal = Elevater2025Goal()
        elevater_goal.mode = Elevater2025Goal.INTAKE
        
        elevator_done = False
        def elevater_done_cb(state, result):
            nonlocal elevator_done
            elevator_done = True
            rospy.loginfo('Elevater action finished')

        self.elevater_client.send_goal(elevater_goal, elevater_done_cb)
        rospy.loginfo("Intaking server - Sent elevator goal")
        r = rospy.Rate(50.0)

        while not elevator_done:
            if self.server.is_preempt_requested():
                rospy.loginfo("2025_intaker_server: preempted")
                self.server.set_preempted()
                return
            r.sleep()

        rospy.loginfo('Elevater action passed check, now intaking')

        roller_goal = Roller2025Goal()
        roller_goal.mode = Roller2025Goal.INTAKE

        roller_done = False
        def roller_done_cb(state, result):
            nonlocal roller_done
            roller_done = True
            rospy.loginfo('Intaking - Roller action finished')
        
        self.roller_client.send_goal(roller_goal, roller_done_cb)
        self.intake_service.call(Command(1.0))

        while not roller_done:
            if self.server.is_preempt_requested():
                rospy.loginfo("2025_intaker_server: preempted")
                self.server.set_preempted()
                return
            r.sleep()

        rospy.loginfo('Intaking - Roller action sent')
        self.feedback.state = self.feedback.IN_ROLLER
        self.server.publish_feedback(self.feedback)
        self.result.success = True
        rospy.loginfo("2025_intaking_server: succeeded")
        self.server.set_succeeded(self.result) 
        

if __name__ == '__main__':
    rospy.init_node('2025_intaking_server')
    server = IntakingServer(rospy.get_name())
    rospy.spin()