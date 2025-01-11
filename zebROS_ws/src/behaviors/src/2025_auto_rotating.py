#!/usr/bin/env python3

import rospy
import tf2_ros
import tf2_geometry_msgs
from math import pi
from frc_msgs.msg import MatchSpecificData
from std_msgs.msg import Float64

RED_TAGS = [
    # (x, y, is_on_reef)
    (13.474, 3.306, True), # 6
    (13.890, 4.026, True), # 7
    (13.474, 4.745, True), # 8
    (12.643, 4.745, True), # 9
    (12.227, 4.026, True), # 10
    (12.643, 3.306, True), # 11
    (16.697, 0.655, False), # 1
    (16.697, 7.396, False), # 2
]
BLUE_TAGS = [
    (4.047, 3.306, True), # 17
    (3.658, 4.026, True), # 18
    (4.074, 4.745, True), # 19
    (4.908, 4.745, True), # 20
    (5.321, 4.026, True), # 21
    (4.908, 3.306, True), # 22
    (0.851, 0.655, False), # 12
    (0.851, 7.396, False), # 13
]

ANGLE_FROM_TAG = {
    # positive x-axis = 0, going counterclockwise
    (13.474, 3.306): 120, # 6
    (13.890, 4.026): 180, # 7
    (13.474, 4.745): 240, # 8
    (12.643, 4.745): 300, # 9
    (12.227, 4.026): 0, # 10
    (12.643, 3.306): 60, # 11
    (16.697, 0.655): 126, # 1
    (16.697, 7.396): 234, # 2
    (4.047, 3.306): 60, # 17
    (3.658, 4.026): 0, # 18
    (4.074, 4.745): 300, # 19
    (4.908, 4.745): 240, # 20
    (5.321, 4.026): 180, # 21
    (4.908, 3.306): 120, # 22
    (0.851, 0.655): 54, # 12
    (0.851, 7.396): 306, # 13
}

has_game_piece = True
def game_piece_callback(msg):
    global has_game_piece
    has_game_piece = msg.has_game_piece

team_color = "green"
def team_color_callback(msg: MatchSpecificData):
    global team_color
    team_color = msg.allianceColor

if __name__ == "__main__":
    rospy.init_node("auto_aligning_2025")
    angle_pub = rospy.Publisher("/teleop/orientation_command", Float64, queue_size=1)
    # game_piece_sub = rospy.Subscriber()
    team_color_sub = rospy.Subscriber("/frcrobot_rio/match_data", MatchSpecificData, team_color_callback)

    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)

    r = rospy.Rate(10)

    # If we can't get our team color, should this still work? idk, here's two versions to choose from
    tags = None
    # while tags is None:
    #     if team_color == "red":
    #         tags = RED_TAGS
    #     elif team_color == "blue":
    #         tags = BLUE_TAGS
    #     else:
    #         rospy.logerr("2025_auto_rotating: Can't find team color, skipping this cycle")
    #     r.sleep()
    if team_color == "red":
        tags = RED_TAGS
    else:
        tags = BLUE_TAGS
    
    angle = None
    while not rospy.is_shutdown():
        try:
            trans = tf_buffer.lookup_transform('map', 'base_link', rospy.Time())
        except:
            rospy.loginfo("2025_auto_rotating: Transform tree not up yet, skipping this cycle")
            r.sleep()
            continue
        x = trans.transform.translation.x
        y = trans.transform.translation.y

        closest_tag = (-1, -1)
        closest_dist_sq = 99999
        for (tag_x, tag_y, is_on_reef) in tags:
            dist_sq = (tag_x - x) ** 2 + (tag_y - y) ** 2

            if (is_on_reef == has_game_piece) or dist_sq < 1: # if we're very close to something, we should stay aligned to that
                if dist_sq < closest_dist_sq:
                    closest_tag = (tag_x, tag_y)
                    closest_dist_sq = dist_sq
        
        angle = ANGLE_FROM_TAG[closest_tag]
        angle *= pi/180
        angle_pub.publish(Float64(data=angle))
        # new_angle = ANGLE_FROM_TAG[closest_tag]
        # if new_angle != angle:
        #     angle = new_angle
        #     angle_pub.publish(Float64(data=angle))

        r.sleep()