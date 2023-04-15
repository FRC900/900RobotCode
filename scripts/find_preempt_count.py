#!/usr/bin/env python3
import rospy
import rosbag
from rospy_message_converter import message_converter
import sys

bagfiles = sys.argv[1:]
preempt = '/align_and_place_grid/cancel'
goal = '/align_and_place_grid/goal'

for bagfile_name in bagfiles:
    bag = rosbag.Bag(bagfile_name)

    preempt_count = 0
    goal_count = 0
    for topic, msg, t in bag.read_messages([preempt, goal]):
        m = message_converter.convert_ros_message_to_dictionary(msg)
        this_time = t.to_sec()

        if topic == preempt:
            preempt_count += 1
        else:
            goal_count += 1
    
    print(f"Bag {bagfile_name} had {preempt_count} align preempts and {goal_count} align goals")
