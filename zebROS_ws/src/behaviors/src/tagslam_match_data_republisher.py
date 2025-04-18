#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from frc_msgs.msg import MatchSpecificData
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse
from behavior_actions.srv import OverrideAllianceColor, OverrideAllianceColorRequest, OverrideAllianceColorResponse
from gpu_apriltag_msgs.srv import SetAllowedTags, SetAllowedTagsRequest, SetAllowedTagsResponse

FIELD_LENGTH = 17.55
RED_TAGS = [6,7,8,9,10,11]
BLUE_TAGS = [17,18,19,20,21,22]

class TagSLAMRepublisher:
    tagslam_alliance = MatchSpecificData.ALLIANCE_COLOR_UNKNOWN
    tagslam_last_x = None
    tagslam_last_time = None
    robot_enabled = True
    manual_override_enabled = False
    manual_alliance = MatchSpecificData.ALLIANCE_COLOR_UNKNOWN
    red_ignored = True
    blue_ignored = True
    last_alliance_color = MatchSpecificData.ALLIANCE_COLOR_UNKNOWN

    LEFT_CAM_IDX = 2 # .10
    RIGHT_CAM_IDX = 0 # .9

    def __init__(self):
        self.tagslam_sub = rospy.Subscriber("/tagslam/odom/body_frc_robot", Odometry, callback=self.tagslam_cb, tcp_nodelay=True, queue_size=1)
        self.match_data_sub = rospy.Subscriber("/frcrobot_rio/match_data_raw", MatchSpecificData, callback=self.match_data_cb, tcp_nodelay=True, queue_size=1)
        self.match_data_pub = rospy.Publisher("/frcrobot_rio/match_data", MatchSpecificData, tcp_nodelay=True, queue_size=1)
        self.override_srv = rospy.Service("enable_manual_override", SetBool, self.manual_override_cb)
        self.set_alliance_srv = rospy.Service("set_alliance_color", OverrideAllianceColor, self.set_alliance_cb)
        self.tag_allow_srvs = [rospy.ServiceProxy("/apriltag_detection_ov2311_10_9_0_9_video0/set_allowed_tags_service", SetAllowedTags), 
                                rospy.ServiceProxy("/apriltag_detection_ov2311_10_9_0_9_video1/set_allowed_tags_service", SetAllowedTags),
                                rospy.ServiceProxy("/apriltag_detection_ov2311_10_9_0_10_video0/set_allowed_tags_service", SetAllowedTags),
                                rospy.ServiceProxy("/apriltag_detection_ov2311_10_9_0_10_video1/set_allowed_tags_service", SetAllowedTags)]
        self.disable_left_tags_srv = rospy.Service("disable_left_tags", SetBool, self.disable_left_tags_cb)
        self.disable_right_tags_srv = rospy.Service("disable_right_tags", SetBool, self.disable_right_tags_cb)
    
    def disable_left_tags_cb(self, req: SetBoolRequest):
        # true = disable
        # ALWAYS REENABLE AFTER PLACING
        if req.data:
            rospy.logwarn("Disabling tags for left camera")
            req = SetAllowedTagsRequest()
            req.allowed_tags = []
            try:
                self.tag_allow_srvs[self.LEFT_CAM_IDX].wait_for_service(timeout=0.01)
                self.tag_allow_srvs[self.LEFT_CAM_IDX].call(req)
            except:
                rospy.logerr_throttle(1.0, f"service {self.tag_allow_srvs[self.LEFT_CAM_IDX].resolved_name} is unavailable, trying again next time")
        else:
            rospy.logwarn("Enabling tags for left camera")
            if self.last_alliance_color == MatchSpecificData.ALLIANCE_COLOR_RED:
                rospy.loginfo("We are red alliance")
                req = SetAllowedTagsRequest()
                req.allowed_tags = RED_TAGS
                try:
                    self.tag_allow_srvs[self.LEFT_CAM_IDX].wait_for_service(timeout=0.01)
                    self.tag_allow_srvs[self.LEFT_CAM_IDX].call(req)
                except:
                    rospy.logerr_throttle(1.0, f"service {self.tag_allow_srvs[self.LEFT_CAM_IDX].resolved_name} is unavailable, trying again next time")
            elif self.last_alliance_color == MatchSpecificData.ALLIANCE_COLOR_BLUE:
                rospy.loginfo("We are blue alliance")
                req = SetAllowedTagsRequest()
                req.allowed_tags = BLUE_TAGS
                try:
                    self.tag_allow_srvs[self.LEFT_CAM_IDX].wait_for_service(timeout=0.01)
                    self.tag_allow_srvs[self.LEFT_CAM_IDX].call(req)
                except:
                    rospy.logerr_throttle(1.0, f"service {self.tag_allow_srvs[self.LEFT_CAM_IDX].resolved_name} is unavailable, trying again next time")
            else:
                rospy.logwarn("Alliance color is unknown, enabling all tags")
                req = SetAllowedTagsRequest()
                req.allowed_tags = RED_TAGS + BLUE_TAGS
                try:
                    self.tag_allow_srvs[self.LEFT_CAM_IDX].wait_for_service(timeout=0.01)
                    self.tag_allow_srvs[self.LEFT_CAM_IDX].call(req)
                except:
                    rospy.logerr_throttle(1.0, f"service {self.tag_allow_srvs[self.LEFT_CAM_IDX].resolved_name} is unavailable, trying again next time")

        return SetBoolResponse()
    
    def disable_right_tags_cb(self, req: SetBoolRequest):
        # true = disable
        # ALWAYS REENABLE AFTER PLACING
        if req.data:
            rospy.logwarn("Disabling tags for right camera")
            req = SetAllowedTagsRequest()
            req.allowed_tags = []
            try:
                self.tag_allow_srvs[self.RIGHT_CAM_IDX].wait_for_service(timeout=0.01)
                self.tag_allow_srvs[self.RIGHT_CAM_IDX].call(req)
            except:
                rospy.logerr_throttle(1.0, f"service {self.tag_allow_srvs[self.RIGHT_CAM_IDX].resolved_name} is unavailable, trying again next time")
        else:
            rospy.logwarn("Enabling tags for right camera")
            if self.last_alliance_color == MatchSpecificData.ALLIANCE_COLOR_RED:
                rospy.loginfo("We are red alliance")
                req = SetAllowedTagsRequest()
                req.allowed_tags = RED_TAGS
                try:
                    self.tag_allow_srvs[self.RIGHT_CAM_IDX].wait_for_service(timeout=0.01)
                    self.tag_allow_srvs[self.RIGHT_CAM_IDX].call(req)
                except:
                    rospy.logerr_throttle(1.0, f"service {self.tag_allow_srvs[self.RIGHT_CAM_IDX].resolved_name} is unavailable, trying again next time")
            elif self.last_alliance_color == MatchSpecificData.ALLIANCE_COLOR_BLUE:
                rospy.loginfo("We are blue alliance")
                req = SetAllowedTagsRequest()
                req.allowed_tags = BLUE_TAGS
                try:
                    self.tag_allow_srvs[self.RIGHT_CAM_IDX].wait_for_service(timeout=0.01)
                    self.tag_allow_srvs[self.RIGHT_CAM_IDX].call(req)
                except:
                    rospy.logerr_throttle(1.0, f"service {self.tag_allow_srvs[self.RIGHT_CAM_IDX].resolved_name} is unavailable, trying again next time")
            else:
                rospy.logwarn("Alliance color is unknown, enabling all tags")
                req = SetAllowedTagsRequest()
                req.allowed_tags = RED_TAGS + BLUE_TAGS
                try:
                    self.tag_allow_srvs[self.RIGHT_CAM_IDX].wait_for_service(timeout=0.01)
                    self.tag_allow_srvs[self.RIGHT_CAM_IDX].call(req)
                except:
                    rospy.logerr_throttle(1.0, f"service {self.tag_allow_srvs[self.RIGHT_CAM_IDX].resolved_name} is unavailable, trying again next time")

        return SetBoolResponse()

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

        if not self.robot_enabled and (self.red_ignored or self.blue_ignored):
            rospy.loginfo("tagslam_match_data_republisher: enabling all reef tags")
            req = SetAllowedTagsRequest()
            req.allowed_tags = RED_TAGS + BLUE_TAGS # needed to find alliance
            success = True
            for tag_allow_srv in self.tag_allow_srvs:
                try:
                    tag_allow_srv.wait_for_service(timeout=0.01)
                    tag_allow_srv.call(req)
                except:
                    success = False
                    rospy.logerr_throttle(1.0, f"tagslam_match_data_republisher: service {tag_allow_srv.resolved_name} is unavailable, trying again next time")
            if success:
                self.red_ignored = False
                self.blue_ignored = False

        prev_alliance = msg.allianceColor
        if self.manual_override_enabled and self.manual_alliance != MatchSpecificData.ALLIANCE_COLOR_UNKNOWN:
            if self.manual_alliance != prev_alliance or self.manual_alliance != self.tagslam_alliance:
                rospy.logerr_throttle(0.5, f"tagslam_match_data_republisher: ***MANUAL OVERRIDE*** FMS alliance color = {self.alliance_color_to_string(prev_alliance)}, TagSLAM alliance color = {self.alliance_color_to_string(self.tagslam_alliance)}. One of these DOES NOT MATCH MANUAL OVERRIDE = {self.alliance_color_to_string(self.manual_alliance)}. Using MANUAL color {self.alliance_color_to_string(self.manual_alliance)}.")
                msg.allianceColor = self.manual_alliance
        elif self.tagslam_alliance != MatchSpecificData.ALLIANCE_COLOR_UNKNOWN:
            if self.tagslam_alliance != prev_alliance:
                rospy.logerr_throttle(0.5, f"tagslam_match_data_republisher: FMS alliance color = {self.alliance_color_to_string(prev_alliance)}. This DOES NOT MATCH TagSLAM-based color = {self.alliance_color_to_string(self.tagslam_alliance)}, based on x = {self.tagslam_last_x} at {self.tagslam_last_time}. Using TagSLAM color {self.alliance_color_to_string(self.tagslam_alliance)}.")
                msg.allianceColor = self.tagslam_alliance
        
        if self.robot_enabled and msg.allianceColor == MatchSpecificData.ALLIANCE_COLOR_RED and (self.red_ignored or not self.blue_ignored):
            rospy.loginfo("tagslam_match_data_republisher: enabling red tags, ignoring blue tags")
            req = SetAllowedTagsRequest()
            req.allowed_tags = RED_TAGS
            success = True
            for tag_allow_srv in self.tag_allow_srvs:
                try:
                    tag_allow_srv.wait_for_service(timeout=0.01)
                    tag_allow_srv.call(req)
                except:
                    success = False
                    rospy.logerr_throttle(1.0, f"service {tag_allow_srv.resolved_name} is unavailable, trying again next time")
            if success:
                self.red_ignored = False
                self.blue_ignored = True
        
        if self.robot_enabled and msg.allianceColor == MatchSpecificData.ALLIANCE_COLOR_BLUE and (self.blue_ignored or not self.red_ignored):
            rospy.loginfo("tagslam_match_data_republisher: enabling blue tags, ignoring red tags")
            req = SetAllowedTagsRequest()
            req.allowed_tags = BLUE_TAGS
            success = True
            for tag_allow_srv in self.tag_allow_srvs:
                try:
                    tag_allow_srv.wait_for_service(timeout=0.01)
                    tag_allow_srv.call(req)
                except:
                    success = False
                    rospy.logerr_throttle(1.0, f"service {tag_allow_srv.resolved_name} is unavailable, trying again next time")
            if success:
                self.red_ignored = True
                self.blue_ignored = False
        
        self.last_alliance_color = msg.allianceColor

        self.match_data_pub.publish(msg)

if __name__ == "__main__":
    rospy.init_node("tagslam_match_data_republisher")
    republisher = TagSLAMRepublisher()  
    rospy.spin()