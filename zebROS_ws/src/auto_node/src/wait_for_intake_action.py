import rospy
from action import Action
from frc_utils.match_data_helper import RobotStatusHelper

from typing import List
from behavior_actions.msg import Placing2025Action, Placing2025Goal, Placing2025Result, Placing2025Feedback
import actionlib
from sensor_msgs.msg import JointState

class WaitForIntakeAction(Action):
    """An action that waits for the intake"""

    def callback(self, msg: JointState):
        if self.__switch_name in msg.name:
            self.__switch = msg.position[msg.name.index(self.__switch_name)]
            #rospy.loginfo(f"Found {self.switch_name} with value {self.switch}")
        else:
            rospy.logwarn_throttle(1.0, f'wait_for_intake_action: {self.__switch_name} not found')
            pass

    def __init__(self):
        self.__joint_states_sub = rospy.Subscriber("/frcrobot_rio/joint_states", JointState, self.callback, tcp_nodelay=True)
        self.__switch_name = "roller_limit_switch"
        self.__early_exit = False

    def start(self):
        self.__early_exit = False

    def update(self):
        pass

    def done(self):
        pass

    def isFinished(self) -> bool:
        return self.__early_exit or self.__switch

    def preempt(self):
        rospy.logwarn("Preempted so we'll stop waiting for intake")
        self.__early_exit = True