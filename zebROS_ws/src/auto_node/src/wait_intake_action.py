import rospy
import actionlib
from action import Action
from datetime import datetime
from typing import List
from behavior_actions.msg import Intaking2024Action, Intaking2024Goal, Intaking2024Feedback, Intaking2024Result

class WaitIntakeAction(Action):
    """Blocks until the intake has had a note hit the diverter limit"""
    def __init__(self, timeout=None):        
        self.__intake_client = actionlib.SimpleActionClient("/intaking/intaking_server_2024", Intaking2024Action)
        self.__finished = False
        self.__timeout = timeout
        self.__start_time = rospy.Time(0)


    def intake_feedback(self, msg: Intaking2024Feedback):
        rospy.loginfo(f"Intake feedback {msg}")
        if msg.note_hit_intake:
            self.__finished = True
            rospy.loginfo("Note hit intake - action finished")

    def start(self):
        rospy.loginfo("Turning on intake in auto mode!")
        intake_action = Intaking2024Goal()
        intake_action.run_until_preempt = True
        intake_action.destination = intake_action.SHOOTER
        self.__intake_client.send_goal(intake_action, feedback_cb=self.intake_feedback)
        self.__start_time = rospy.Time.now()

    def update(self):
        # timeout might be none
        if self.__timeout:
            if rospy.Time.now() - self.__start_time > rospy.Duration(self.__timeout):
                rospy.logwarn("Wait for intake action timeout hit, moving on")
                self.__finished = True

    def done(self):
        pass

    def isFinished(self) -> bool:
        return self.__finished

    def preempt(self):
        rospy.logwarn("Preempt called for wait intake action, stopping intake")
        self.__intake_client.cancel_goals_at_and_before_time(rospy.Time.now())
        self.__finished = True
