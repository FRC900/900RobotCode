import rospy
import actionlib
from action import Action
from datetime import datetime
from typing import List
from behavior_actions.msg import Intaking2024Action, Intaking2024Goal, Intaking2024Feedback, Intaking2024Result

class IntakeAction(Action):
    """Turns on the intake forever"""
    def __init__(self, off=False):        
        self.__intake_client = actionlib.SimpleActionClient("/intaking/intaking_server_2024", Intaking2024Action)
        self.__finished = False
        self.__off = off

    def start(self):
        if self.__off:
            rospy.loginfo("Turning off intake!")
            self.__intake_client.cancel_goals_at_and_before_time(rospy.Time.now())
        else:
            rospy.loginfo("Turning on intake in auto mode!")
            intake_action = Intaking2024Goal()
            intake_action.run_until_preempt = True
            intake_action.destination = intake_action.SHOOTER
            self.__intake_client.send_goal(intake_action)
        self.__finished = True

    def update(self):
        pass

    def done(self):
        pass

    def isFinished(self) -> bool:
        return self.__finished

    def preempt(self):
        rospy.logwarn("Preempt called for intake action, stopping intake")
        self.__intake_client.cancel_goals_at_and_before_time(rospy.Time.now())
        self.__finished = True
