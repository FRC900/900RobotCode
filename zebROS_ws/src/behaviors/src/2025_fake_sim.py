#!/usr/bin/env python3

import rospy
import tf2_ros

from frc_msgs.msg import MatchSpecificData
from talon_state_msgs.msg import TalonFXProState
# from sensor_msgs.msg import JointState
from ros_control_boilerplate.srv import LineBreakSensors

RED_TAGS = [
    # (x, y)
    (16.697, 0.655), # 1
    (16.697, 7.396), # 2
]
BLUE_TAGS = [
    (0.851, 0.655), # 12
    (0.851, 7.396), # 13
]

def team_color_callback(msg: MatchSpecificData):
    global team_color
    team_color = msg.allianceColor

def talon_states_callback(states: TalonFXProState):
    global state
    for idx in range(len(states.name)):
        if states.name[idx] == "intake":
            intake_output = states.control_output[idx]
        elif states.name[idx] == "roller":
            roller_output = states.control_output[idx]

    if roller_output > 0.1:
        if intake_output > 0.1:
            state = 1 # intaking
        else:
            state = 2 # placing
    else:
        state = 0 # neither

# def game_piece_callback(data):
#     global has_game_piece
#     if switch_name in data.name:
#         has_game_piece = data.position[data.name.index(switch_name)] != 0
#     else:
#         rospy.logwarn_throttle(1.0, f'2025_intaking_server: {switch_name} not found')
#         pass

def simulate_intaking():
    rospy.loginfo("intake success!")
    try:
        linebreak_service(0, "intake_far_limit_switch", True)
        linebreak_service(0, "intake_close_limit_switch", True)
        rospy.sleep(0.3)
        linebreak_service(0, "intake_far_limit_switch", False)
        linebreak_service(0, "intake_close_limit_switch", False)
        linebreak_service(0, "roller_limit_switch", True)
    except rospy.ServiceException as e:
        rospy.logerr(f"linebreak_service call failed: {e}")

def simulate_placing():
    rospy.loginfo("placing success!")
    try:
        linebreak_service(0, "roller_limit_switch", False)
    except rospy.ServiceException as e:
        rospy.logerr(f"linebreak_service call failed: {e}")


if __name__ == "__main__":
    rospy.init_node("fake_sim_2025")

    threshold = 2.0
    time_req = rospy.Duration.from_sec(1.0)

    team_color = -1 #green!
    team_color_sub = rospy.Subscriber("/frcrobot_rio/match_data", MatchSpecificData, team_color_callback)

    intake_output = 0
    roller_output = 0
    state = 0
    talon_states = rospy.Subscriber("/frcrobot_jetson/talonfxpro_states", TalonFXProState, talon_states_callback, tcp_nodelay=True)

    # has_game_piece = False
    # game_piece_sub = rospy.Subscriber("/frcrobot_rio/joint_states", JointState, game_piece_callback)

    linebreak_service = rospy.ServiceProxy('/frcrobot_rio/linebreak_service_set', LineBreakSensors)

    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)

    location = None
    enter_time = None
    r = rospy.Rate(60)
    while not rospy.is_shutdown():
        if team_color == MatchSpecificData.ALLIANCE_COLOR_RED:
            tags = RED_TAGS
        else:
            tags = BLUE_TAGS
        
        try:
            trans = tf_buffer.lookup_transform('map', 'base_link', rospy.Time())
        except:
            rospy.loginfo("2025_fake_sim: Transform tree not up yet, skipping this cycle")
            r.sleep()
            continue
        x = trans.transform.translation.x
        y = trans.transform.translation.y

        near_station = False
        if location is None:
            for (tag_x, tag_y) in tags:
                if (tag_x - x) ** 2 + (tag_y - y) ** 2 < threshold ** 2:
                    near_station = True
                    enter_time = rospy.Time.now()
                    location = (tag_x, tag_y)
                    break
        else:
            if (location[0] - x) ** 2 + (location[1] - y) ** 2 > threshold ** 2:
                location = None

        if state != 0:
            if state == 1:
                while rospy.Time.now() - enter_time < time_req:
                    try:
                        trans = tf_buffer.lookup_transform('map', 'base_link', rospy.Time())
                    except:
                        rospy.loginfo("2025_fake_sim: Transform tree not up yet, skipping this cycle")
                        r.sleep()
                        continue
                    x = trans.transform.translation.x
                    y = trans.transform.translation.y

                    if (location[0] - x) ** 2 + (location[1] - y) ** 2 > threshold ** 2:
                        location = None
                        break
                    r.sleep()
                
                if location is not None:
                    simulate_intaking()
                    rospy.sleep(0.5) # if we don't do this it flips the switches multiple times
            else:
                rospy.sleep(0.3)
                simulate_placing()
                rospy.sleep(0.5) # if we don't do this it flips the switches multiple times
        r.sleep()