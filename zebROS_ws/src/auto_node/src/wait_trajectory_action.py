import rospy
from action import Action
import actionlib.simple_action_client
from frc_utils.match_data_helper import RobotStatusHelper

from typing import List
from path_follower_msgs.msg import PathActionFeedback
from auto_node_msgs.msg import PathGoalArray # have the path to be sent to the path follower
import actionlib

class WaitTrajectoryAction(Action):
    """An action that waits for the current trajectory to reach a certain percent complete
       Uses the path follower feedback topic to determine this. 
    """

    def __init__(self, percent_complete):
        self.__path_follower_feedback_sub = rospy.Subscriber("/path_follower/path_follower_server/feedback", PathActionFeedback, self.feedback_cb, tcp_nodelay=True)
        self.__latest_feedback: PathActionFeedback = None
        if not percent_complete:
            rospy.logerr("Must provide percent complete in wait for trajectory")
            exit(1)
        self.__percent_complete = percent_complete
        self.__finished = False

    def feedback_cb(self, msg: PathActionFeedback):
        rospy.logwarn_once("Started to get feedback message for the path follower")
        self.__latest_feedback = msg

    def start(self):
        rospy.loginfo(f"Waiting for path to reach {self.__percent_complete}")

    def update(self):
        if not self.__latest_feedback:
            rospy.logwarn_throttle(1, "No feedback found in wait for trajectory")
            return
        if self.__latest_feedback.feedback.percent_complete >= self.__percent_complete:
            rospy.loginfo(f"Path has reached at least {self.__percent_complete}, moving on! Last feedback was {self.__latest_feedback}")
            self.__finished = True

    def done(self):
        pass

    def isFinished(self) -> bool:
        return self.__finished

    def preempt(self):
        rospy.logwarn("Preempt called for wait trajectory action")
        # will cancel a couple times when there are more than 1 drive trajectory actions but should be ok
        self.__finished = True

