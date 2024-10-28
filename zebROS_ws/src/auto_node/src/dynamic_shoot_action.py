import rospy
from action import Action
import actionlib.simple_action_client
from frc_utils.match_data_helper import RobotStatusHelper

from typing import List
from behavior_actions.msg import Shooting2024Action, Shooting2024Goal, Shooting2024Result, Shooting2024Feedback
from behavior_actions.msg import AutoAlignSpeaker
import actionlib

class DynamicShootAction(Action):
    """An action that shoots without aligning using the current distance from the subwoofer"""

    def __init__(self, subwoofer=False):
        # do need to check how potentially having multiple action clients on the same topic works
        # conflicting goals could be bad, but seems the same as one client sending two goals
        self.__shooting_client = actionlib.SimpleActionClient("/shooting/shooting_server_2024", Shooting2024Action)
        if not self.__shooting_client.wait_for_server(rospy.Duration(5)):
            rospy.logerr("Shooting clinet  not up after 5 seconds, exiting")
            exit(1)

        self.__subwoofer = subwoofer
        self.__speaker_dist = None
        self.__done = False
        self.__dist_angle_sub = rospy.Subscriber("/speaker_align/dist_and_ang", AutoAlignSpeaker, self.distance_cb, tcp_nodelay=True)
     
    def distance_cb(self, msg: AutoAlignSpeaker):
        self.__speaker_dist = msg.distance

    def done_cb(self, status: Shooting2024Feedback, result: Shooting2024Result):
        rospy.loginfo("Dynamic Shooting done")
        self.__done = True

    def start(self):
        rospy.loginfo(f"Running dynamic shooting step for auto, subwoofer shot={self.__subwoofer}")
        shooting_goal: Shooting2024Goal = Shooting2024Goal()
        shooting_goal.leave_spinning = True # want to keep going 
        shooting_goal.setup_only = False # actually want to shoot something
        shooting_goal.mode = shooting_goal.SPEAKER
        if self.__subwoofer:
            shooting_goal.distance = 1.43 # subwoofer
        else:
            shooting_goal.distance = self.__speaker_dist
        rospy.loginfo(f"Shooting from distance {shooting_goal.distance}")
        
        self.__shooting_client.send_goal(shooting_goal, done_cb=self.done_cb)

    def update(self):
        pass

    def done(self):
        pass

    def isFinished(self) -> bool:
        return self.__done

    def preempt(self):
        rospy.logwarn("Preempt called for dynamic shoot action, stopping shooter")
        self.__shooting_client.cancel_goals_at_and_before_time(rospy.Time.now())
        self.__done = True

