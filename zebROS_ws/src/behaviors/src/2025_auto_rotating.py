#!/usr/bin/env python3

import actionlib.simple_action_client
import rospy
import tf2_ros
import tf2_geometry_msgs
from math import pi
from frc_msgs.msg import MatchSpecificData
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from std_srvs.srv import SetBool, SetBoolResponse
import actionlib 
from behavior_actions.msg import Intaking2025Action, Intaking2025Goal

# needs to NOT RUN DURING PATH FOLLOWING
# need to stop intaking when away from coral station
# could publish orient strafing control effort to a cmd vel topic that has lower priority than teleop so we autorotate when not driving

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

has_game_piece = False
def game_piece_callback(data):
    global has_game_piece
    if switch_name in data.name:
        has_game_piece = data.position[data.name.index(switch_name)]
    else:
        rospy.logwarn_throttle(1.0, f'2025_intaking_server: {switch_name} not found')
        pass
    
team_color = -1 #green!
is_auto = False
def team_color_callback(msg: MatchSpecificData):
    global team_color, is_auto
    team_color = msg.allianceColor
    is_auto = msg.Autonomous

should_run = True
def handle_service(req):
    global should_run
    should_run = req.data
    return SetBoolResponse(success=True)


if __name__ == "__main__":
    rospy.init_node("auto_rotating_2025")
    angle_pub = rospy.Publisher("/teleop/orientation_command", Float64, queue_size=1)

    switch_name = rospy.get_param("switch_name")
    too_close_zone = rospy.get_param("too_close_zone")

    game_piece_sub = rospy.Subscriber("/frcrobot_rio/joint_states", JointState, game_piece_callback, tcp_nodelay=True)
    team_color_sub = rospy.Subscriber("/frcrobot_rio/match_data", MatchSpecificData, team_color_callback)
    intaking_client = actionlib.SimpleActionClient("/intaking/intaking_server_2025", Intaking2025Action)
    rospy.loginfo("waiting for intaking server")
    intaking_client.wait_for_server()
    rospy.loginfo("Intake client created")
    service = rospy.Service("toggle_auto_rotate", SetBool, handle_service)

    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)

    r = rospy.Rate(60)

    tags = None
    angle = None
    intake_running = False
    while not rospy.is_shutdown():
        r.sleep()

        if team_color == MatchSpecificData.ALLIANCE_COLOR_RED:
            tags = RED_TAGS
        else:
            tags = BLUE_TAGS
            


        try:
            trans = tf_buffer.lookup_transform('map', 'base_link', rospy.Time())
        except:
            rospy.loginfo_throttle(1, "2025_auto_rotating: Transform tree not up yet, skipping this cycle")
            r.sleep()
            continue
        x = trans.transform.translation.x
        y = trans.transform.translation.y

        closest_dist_sq = 99999
        reef_distances = []
        coral_distances = []
        
        for (tag_x, tag_y, is_on_reef) in tags:
            dist_sq = (tag_x - x) ** 2 + (tag_y - y) ** 2
            if is_on_reef:
                reef_distances.append((dist_sq, tag_x, tag_y))
            else:
                coral_distances.append((dist_sq, tag_x, tag_y))

        closest_reef = min(reef_distances, key=lambda x: x[0])
        closest_coral =  min(coral_distances, key=lambda x: x[0])
        if has_game_piece:
            closest_reef_dist, tag_x, tag_y = closest_reef
            closest_tag = (tag_x, tag_y)
            # intake should stop automatically once we get a game piece
            if intake_running:
                rospy.loginfo("Not running intake anymore")
                # intaking_client.cancel_goals_at_and_before_time(rospy.Time.now())
                intake_running = False
        else: 
            closest_coral_dist, tag_x, tag_y = closest_coral
            closest_tag = (tag_x, tag_y)
            #rospy.loginfo(f"closest coral dist {closest_coral_dist}")
            if closest_coral_dist < 2.0 and not intake_running:
                rospy.loginfo("Running intake automatically")
                intake_running = True
                intake_goal = Intaking2025Goal()
                intaking_client.send_goal(intake_goal)
            elif closest_coral_dist >= 2.0:
                if intake_running:
                    pass
                    #intaking_client.cancel_goals_at_and_before_time(rospy.Time.now())
                intake_running = False
        #rospy.loginfo(f"{tag_x, tag_y}")
        dist_sq = min(closest_coral[0], closest_reef[0])

        if dist_sq < too_close_zone ** 2: # if we're very close to something, we should stay aligned to that
            closest_tag = (tag_x, tag_y)
            closest_dist_sq = dist_sq
        
        if is_auto or not should_run:
            continue

        angle = ANGLE_FROM_TAG[closest_tag]
        angle *= pi/180
        angle_pub.publish(Float64(data=angle))