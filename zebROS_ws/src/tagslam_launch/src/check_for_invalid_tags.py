#!/usr/bin/env python3

import rospy
from apriltag_msgs.msg import ApriltagArrayStamped
from frc_msgs.msg import MatchSpecificData

enabled = False
match_time = -1
alliance_color = MatchSpecificData.ALLIANCE_COLOR_RED

valid_tags = { MatchSpecificData.ALLIANCE_COLOR_UNKNOWN: {},
               MatchSpecificData.ALLIANCE_COLOR_RED: {6, 7, 8, 9, 10, 11},
               MatchSpecificData.ALLIANCE_COLOR_BLUE: {17, 18, 19, 20, 21, 22} }

def match_data_callback(msg : MatchSpecificData):
    global enabled
    global match_time
    global alliance_color

    enabled = msg.Enabled
    match_time = msg.matchTimeRemaining
    alliance_color = msg.allianceColor
    print(f'{alliance_color} {enabled} {match_time}')

def tag_callback(msg: ApriltagArrayStamped, args):
    global enabled
    global match_time
    global alliance_color

    # Check if the message contains invalid tags
    for tag in msg.apriltags:
        if tag.id not in valid_tags[alliance_color]:
            rospy.logwarn(f"{args} : Invalid tag detected: ID {tag.id}, match time {match_time}, enabled {enabled}, alliance color {alliance_color}") 
            return

rospy.init_node('check_for_invalid_tags')
tag_sub_0 = rospy.Subscriber('/apriltag_detection_ov2311_10_9_0_9_video0/tags', ApriltagArrayStamped, tag_callback, "9_0")
tag_sub_1 = rospy.Subscriber('/apriltag_detection_ov2311_10_9_0_9_video1/tags', ApriltagArrayStamped, tag_callback, "9_1")
tag_sub_2 = rospy.Subscriber('/apriltag_detection_ov2311_10_9_0_10_video0/tags', ApriltagArrayStamped, tag_callback, "10_0")
tag_sub_3 = rospy.Subscriber('/apriltag_detection_ov2311_10_9_0_10_video1/tags', ApriltagArrayStamped, tag_callback, "10_1")
match_data_sub = rospy.Subscriber('/frcrobot_rio/match_data', MatchSpecificData, match_data_callback)
rospy.spin()