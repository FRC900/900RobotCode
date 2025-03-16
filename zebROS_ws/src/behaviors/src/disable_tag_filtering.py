#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from frc_msgs.msg import MatchSpecificData
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse
from behavior_actions.srv import OverrideAllianceColor, OverrideAllianceColorRequest, OverrideAllianceColorResponse
from gpu_apriltag_msgs.srv import SetAllowedTags, SetAllowedTagsRequest, SetAllowedTagsResponse
import time

FIELD_LENGTH = 17.55
RED_TAGS = [6,7,8,9,10,11]
BLUE_TAGS = [17,18,19,20,21,22]
ALL_TAGS = list(range(33))

rospy.init_node("tag_filter_disabler")
tag_allow_srvs = [rospy.ServiceProxy("/apriltag_detection_ov2311_10_9_0_9_video0/set_allowed_tags_service", SetAllowedTags), 
                  rospy.ServiceProxy("/apriltag_detection_ov2311_10_9_0_9_video1/set_allowed_tags_service", SetAllowedTags),
                  rospy.ServiceProxy("/apriltag_detection_ov2311_10_9_0_10_video0/set_allowed_tags_service", SetAllowedTags),
                  rospy.ServiceProxy("/apriltag_detection_ov2311_10_9_0_10_video1/set_allowed_tags_service", SetAllowedTags)]

time.sleep(1.0)

success = True

req = SetAllowedTagsRequest()
req.allowed_tags = ALL_TAGS
for tag_allow_srv in tag_allow_srvs:
    try:
        tag_allow_srv.wait_for_service(timeout=0.01)
        tag_allow_srv.call(req)
    except:
        success = False
        rospy.logerr_throttle(1.0, f"tagslam_match_data_republisher: service {tag_allow_srv.resolved_name} is unavailable, trying again next time")

rospy.loginfo("Disabled tag filtering")

rospy.spin()