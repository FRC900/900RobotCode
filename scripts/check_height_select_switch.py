import os
import rospy
import rosbag
from rospy_message_converter import message_converter
import sys

#Bagfile name from command line
bagfile_name = sys.argv[1]

bag = rosbag.Bag(bagfile_name)

top = '/frcrobot_rio/button_box_states'
match = '/frcrobot_rio/match_data'
place = '/align_and_place_grid/goal'
out = "/rosout"
auto = False

last_match_time = 0

for topic, msg, t in bag.read_messages([top, match, place, out]):
    m = message_converter.convert_ros_message_to_dictionary(msg)
    if topic == top:
        #print(m)

        if m["heightSelectSwitchLeftPress"]:
            print(f"Left press at {t.to_sec()} = {last_match_time}. auto = {auto}")
        if m["heightSelectSwitchRightPress"]:
            print(f"Right press at {t.to_sec()} = {last_match_time}. auto = {auto}")
        if m["gridSelectConeLeftPress"]:
            print(f"Left cone press at {t.to_sec()} = {last_match_time}. auto = {auto}")
        if m["gridSelectConeLeftPress"] and m["gridSelectConeLeftButton"]:
            print(f"Left cone press AND button at {t.to_sec()} = {last_match_time}. auto = {auto}")
        if m["gridSelectConeRightPress"]:
            print(f"Right cone press at {t.to_sec()} = {last_match_time}. auto = {auto}")
        if m["gridSelectCubePress"]:
            print(f"Right cube press at {t.to_sec()} = {last_match_time}. auto = {auto}")
    elif topic == match:
        last_match_time = m["matchTimeRemaining"]
        auto = m["Autonomous"]
    elif topic == out:
        if "Sending align to goal" in m["msg"] or "AutoPlaceState" in m["msg"]:
            print(f'{m["msg"]} at {t.to_sec()} = {last_match_time}. auto = {auto}')
    else:
        print(f"Auto place at {t.to_sec()} = {last_match_time}. auto = {auto}")
        # print(f"{msg}")