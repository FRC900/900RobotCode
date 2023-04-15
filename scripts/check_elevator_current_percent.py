#!/usr/bin/env python3
import rospy
import rosbag
from rospy_message_converter import message_converter
import sys

bagfile_name = sys.argv[1]

bag = rosbag.Bag(bagfile_name)

talon = '/frcrobot_jetson/talon_states'

for topic, msg, t in bag.read_messages([talon]):
    m = message_converter.convert_ros_message_to_dictionary(msg)
    this_time = t.to_sec()

    pos = m['position'][5]
    speed = m['speed'][5]
    percent = m['motor_output_percent'][5]
    
    if pos > 1.25 and abs(speed) < 0.01:
        print(f"{m['name'][5]} {percent * 100}% @ {m['position'][5]}")