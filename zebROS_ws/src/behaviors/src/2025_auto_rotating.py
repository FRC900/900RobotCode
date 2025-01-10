#!/usr/bin/env python3

import rospy
from frc_msgs.msg import MatchSpecificData
from std_msgs.msg import Int32

BLUE_TAGS = [
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
RED_TAGS = [
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
    (16.697, 0.655): 306, # 1
    (16.697, 7.396): 54, # 2
    (4.047, 3.306): 60, # 17
    (3.658, 4.026): 0, # 18
    (4.074, 4.745): 300, # 19
    (4.908, 4.745): 240, # 20
    (5.321, 4.026): 180, # 21
    (4.908, 3.306): 120, # 22
    (0.851, 0.655): 234, # 12
    (0.851, 7.396): 126, # 13
}

x = 0
y = 0
def position_callback(msg):
    global x
    global y
    x = msg.x
    y = msg.y

has_game_piece = True
def game_piece_callback(msg):
    global has_game_piece
    has_game_piece = msg.has_game_piece

team_color = "green"
def team_color_callback(msg):
    global team_color
    team_color = msg.allianceColor

if __name__ == "__main__":
    rospy.init_node("2025_auto_align")
    angle_pub = rospy.Publisher("/auto_align_angle", Int32, queue_size=1)
    position_sub = rospy.Subscriber()
    game_piece_sub = rospy.Subscriber()
    team_color_sub = rospy.Subscriber("/frcrobot_rio/match_data", MatchSpecificData, team_color_callback)

    if team_color == "red":
        tags = RED_TAGS
    else:
        tags = BLUE_TAGS

    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        closest_tag = (-1, -1)
        closest_dist_sq = 99999
        for (tag_x, tag_y, is_on_reef) in tags:
            dist_sq = (tag_x - x) ** 2 + (tag_y - y) ** 2

            if is_on_reef == has_game_piece or dist_sq < 0.25 ** 2: # if we're very close to something, we should stay aligned to that
                if dist_sq < closest_dist_sq:
                    closest_tag = (tag_x, tag_y)
                    closest_dist_sq = dist_sq
        
        angle = ANGLE_FROM_TAG[closest_tag]
        angle_pub.publish(Int32(data=angle))

        r.sleep()