#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from frc_msgs.msg import MatchSpecificData
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse
from behavior_actions.srv import OverrideAllianceColor, OverrideAllianceColorRequest, OverrideAllianceColorResponse

FIELD_LENGTH = 17.55

class TagSLAMRepublisher:
    tagslam_alliance = MatchSpecificData.ALLIANCE_COLOR_UNKNOWN
    tagslam_last_x = None
    tagslam_last_time = None
    robot_enabled = True
    manual_override_enabled = False
    manual_alliance = MatchSpecificData.ALLIANCE_COLOR_UNKNOWN

    def __init__(self):
        self.tagslam_sub = rospy.Subscriber("/tagslam/odom/body_frc_robot", Odometry, callback=self.tagslam_cb, tcp_nodelay=True, queue_size=1)
        self.match_data_sub = rospy.Subscriber("/frcrobot_rio/match_data_raw", MatchSpecificData, callback=self.match_data_cb, tcp_nodelay=True, queue_size=1)
        self.match_data_pub = rospy.Publisher("/frcrobot_rio/match_data", MatchSpecificData, tcp_nodelay=True, queue_size=1)
        self.override_srv = rospy.Service("enable_manual_override", SetBool, self.manual_override_cb)
        self.set_alliance_srv = rospy.Service("set_alliance_color", OverrideAllianceColor, self.set_alliance_cb)
    
    def manual_override_cb(self, req: SetBoolRequest):
        if req.data:
            rospy.logwarn("tagslam_match_data_republisher: ENABLING MANUAL OVERRIDE. HERE BE DRAGONS. (or unicorns?)")
            rospy.logwarn(f"tagslam_match_data_republisher: MANUAL OVERRIDE ENABLED, ALLIANCE COLOR IS CURRENTLY {self.alliance_color_to_string(self.manual_alliance)}")
            self.manual_override_enabled = True
        else:
            rospy.logwarn("tagslam_match_data_republisher: disabling manual override")
            self.manual_override_enabled = False
        return SetBoolResponse()
    
    def set_alliance_cb(self, req: OverrideAllianceColorRequest):
        self.manual_alliance = req.allianceColor
        if self.manual_override_enabled:
            rospy.logwarn(f"tagslam_match_data_republisher: MANUAL OVERRIDE ENABLED, ALLIANCE COLOR SET TO {self.alliance_color_to_string(self.manual_alliance)}")
        return OverrideAllianceColorResponse()

    def tagslam_cb(self, msg: Odometry):
        if not self.robot_enabled:
            if msg.pose.pose.position.x <= FIELD_LENGTH/2:
                # rospy.loginfo("valid tagslam blue")
                self.tagslam_alliance = MatchSpecificData.ALLIANCE_COLOR_BLUE
                self.tagslam_last_x = msg.pose.pose.position.x
                self.tagslam_last_time = msg.header.stamp
            elif msg.pose.pose.position.x <= FIELD_LENGTH:
                # rospy.loginfo("valid tagslam red")
                self.tagslam_alliance = MatchSpecificData.ALLIANCE_COLOR_RED
                self.tagslam_last_x = msg.pose.pose.position.x
                self.tagslam_last_time = msg.header.stamp
    
    def alliance_color_to_string(self, color: int):
        if color == MatchSpecificData.ALLIANCE_COLOR_RED:
            return "red"
        elif color == MatchSpecificData.ALLIANCE_COLOR_BLUE:
            return "blue"
        elif color == MatchSpecificData.ALLIANCE_COLOR_UNKNOWN:
            return "unknown"
        else:
            return "???????"
    
    def match_data_cb(self, msg: MatchSpecificData):
        self.robot_enabled = msg.Enabled

        if self.manual_override_enabled and self.manual_alliance != MatchSpecificData.ALLIANCE_COLOR_UNKNOWN:
            prev_alliance = msg.allianceColor
            if self.manual_alliance != prev_alliance or self.manual_alliance != self.tagslam_alliance:
                rospy.logerr(f"tagslam_match_data_republisher: ***MANUAL OVERRIDE*** FMS alliance color = {self.alliance_color_to_string(prev_alliance)}, TagSLAM alliance color = {self.alliance_color_to_string(self.tagslam_alliance)}. One of these DOES NOT MATCH MANUAL OVERRIDE = {self.alliance_color_to_string(self.manual_alliance)}. Using MANUAL color {self.alliance_color_to_string(self.manual_alliance)}.")
                msg.allianceColor = self.manual_alliance
        elif self.tagslam_alliance != MatchSpecificData.ALLIANCE_COLOR_UNKNOWN:
            prev_alliance = msg.allianceColor
            if self.tagslam_alliance != prev_alliance:
                rospy.logerr(f"tagslam_match_data_republisher: FMS alliance color = {self.alliance_color_to_string(prev_alliance)}. This DOES NOT MATCH TagSLAM-based color = {self.alliance_color_to_string(self.tagslam_alliance)}, based on x = {self.tagslam_last_x} at {self.tagslam_last_time}. Using TagSLAM color {self.alliance_color_to_string(self.tagslam_alliance)}.")
                msg.allianceColor = self.tagslam_alliance 
        self.match_data_pub.publish(msg)
 
if __name__ == "__main__":
    rospy.init_node("tagslam_match_data_republisher")
    republisher = TagSLAMRepublisher()  
    rospy.spin()