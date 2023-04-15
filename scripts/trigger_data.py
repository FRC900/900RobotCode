#!/usr/bin/env python3
import rospy
import rosbag
from rospy_message_converter import message_converter
import sys

bagfile_name = sys.argv[1]

bag = rosbag.Bag(bagfile_name)

match = '/frcrobot_rio/match_data'
joystick = '/frcrobot_rio/joystick_states1'
lastTime = 0
lastBumperState = False

for topic, msg, t in bag.read_messages([match, joystick]):
    m = message_converter.convert_ros_message_to_dictionary(msg)
    this_time = t.to_sec()

    if topic == match:
        lastTime = m['matchTimeRemaining']
    else:
        if m['bumperLeftPress']:
            print(f"Left bumper pressed at {lastTime}")
        if m['bumperLeftRelease']:
            print(f"Left bumper released at {lastTime}")
        if m['bumperLeftButton'] != lastBumperState:
            print(f"Left bumper changed state to {m['bumperLeftButton']} at {lastTime}")
            lastBumperState = m['bumperLeftButton']