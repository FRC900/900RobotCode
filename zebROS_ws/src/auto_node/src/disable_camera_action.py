#!/usr/bin/env python3
import rospy
from action import Action
import actionlib.simple_action_client
from frc_utils.match_data_helper import RobotStatusHelper

from typing import List
from behavior_actions.msg import Placing2025Action, Placing2025Goal, Placing2025Result, Placing2025Feedback
import actionlib
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse

class DisableCameraAction(Action):
    """An action that disables a camera"""

    def __init__(self, disable_left = False, disable_right = False):
        self.__disable_left = disable_left
        self.__disable_right = disable_right
        self.disable_left_tags_srv = rospy.ServiceProxy("/tagslam_match_data_republisher/disable_left_tags", SetBool)
        self.disable_right_tags_srv = rospy.ServiceProxy("/tagslam_match_data_republisher/disable_right_tags", SetBool)
        self.__done = False

    def start(self):
        rospy.loginfo("Running disable tags step for auto")
        self.__done = False

        if self.__disable_left:
            rospy.loginfo("Disabling left camera (hope you're scoring on the right!)")
        else:
            rospy.loginfo("Reenabling left camera")
        req = SetBoolRequest()
        req.data = self.__disable_left
        try:
            self.disable_left_tags_srv.wait_for_service(timeout=0.01)
            self.disable_left_tags_srv.call(req)
        except:
            rospy.logwarn("couldn't disable tags, services down?")

        if self.__disable_right:
            rospy.loginfo("Disabling right camera (hope you're scoring on the left!)")
        else:
            rospy.loginfo("Reenabling right camera")
        req = SetBoolRequest()
        req.data = self.__disable_right
        try:
            self.disable_right_tags_srv.wait_for_service(timeout=0.01)
            self.disable_right_tags_srv.call(req)
        except:
            rospy.logwarn("couldn't disable tags, services down?")
        
        self.__done = True

    def update(self):
        pass

    def done(self):
        pass

    def isFinished(self) -> bool:
        return self.__done

    def preempt(self):
        rospy.logwarn("Preempt called for disable camera action, reenabling cameras")
        # Enable all tags
        rospy.loginfo("Enabling all tags, aka disable = false")
        req = SetBoolRequest()
        req.data = False
        try:
            self.disable_left_tags_srv.wait_for_service(timeout=0.01)
            self.disable_left_tags_srv.call(req)
            self.disable_right_tags_srv.wait_for_service(timeout=0.01)
            self.disable_right_tags_srv.call(req)
        except:
            rospy.logwarn("womp womp couldn't reenable tags, services down?")

