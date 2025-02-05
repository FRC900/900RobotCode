import rospy
from action import Action
import actionlib.simple_action_client
from frc_utils.match_data_helper import RobotStatusHelper

from typing import List
from behavior_actions.msg import Placing2025Action, Placing2025Goal, Placing2025Result, Placing2025Feedback
import actionlib

class PlacingAction(Action):
    """An action that places"""

    def __init__(self):
        # do need to check how potentially having multiple action clients on the same topic works
        # conflicting goals could be bad, but seems the same as one client sending two goals
        self.__placing_client = actionlib.SimpleActionClient("placing/2025_placing_server", Placing2025Action)
        if not self.__placing_client.wait_for_server(rospy.Duration(5)):
            rospy.logerr("placing client  not up after 5 seconds, exiting")
            exit(1)
        self.__done = False
     
    

    def done_cb(self, status: Placing2025Feedback, result: Placing2025Result):
        rospy.loginfo("Placing done")
        self.__done = True

    def start(self):
        rospy.loginfo("Running placing step for auto")
        placing_goal: Placing2025Goal = Placing2025Goal()
        placing_goal.mode = 0 #level 4 coral placing 
        
        self.__placing_client.send_goal(placing_goal, done_cb=self.done_cb)

    def update(self):
        pass

    def done(self):
        pass

    def isFinished(self) -> bool:
        return self.__done

    def preempt(self):
        rospy.logwarn("Preempt called for placing action, stopping placer")
        self.__placing_client.cancel_goals_at_and_before_time(rospy.Time.now())
        self.__done = True

