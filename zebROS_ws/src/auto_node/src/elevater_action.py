import rospy
from action import Action
import actionlib.simple_action_client
from frc_utils.match_data_helper import RobotStatusHelper

from typing import List
from behavior_actions.msg import Elevater2025Action, Elevater2025Goal, Elevater2025Result, Elevater2025Feedback
import actionlib

class ElevaterAction(Action):
    """An action that brings the elevator to the stow position"""

    def __init__(self, SetupBool = False):
        # do need to check how potentially having multiple action clients on the same topic works
        # conflicting goals could be bad, but seems the same as one client sending two goals
        self.__elevater_client = actionlib.SimpleActionClient("/elevater/elevater_server_2025", Elevater2025Action)
        self.__SetupBool = SetupBool
        if not self.__elevater_client.wait_for_server(rospy.Duration(5)):
            rospy.logerr("elevater client  not up after 5 seconds, exiting")
            exit(1)
        self.__done = False

    def done_cb(self, status: Elevater2025Feedback, result: Elevater2025Result):
        rospy.loginfo("Elevater done")
        self.__done = True

    def start(self):
        rospy.loginfo("Running elevater step for auto, sending to stow")
        self.__done = False
        elevater_goal: Elevater2025Goal = Elevater2025Goal()
        elevater_goal.mode = elevater_goal.STOW
        
        self.__elevater_client.send_goal(elevater_goal, done_cb=self.done_cb)

    def update(self):
        pass

    def done(self):
        pass

    def isFinished(self) -> bool:
        return self.__done

    def preempt(self):
        rospy.logwarn("Preempt called for elevating action, stopping elevater")
        self.__elevater_client.cancel_goals_at_and_before_time(rospy.Time.now())
        self.__done = True

