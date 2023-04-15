import os
import rospy
import rosbag
from rospy_message_converter import message_converter
import sys

#Bagfile name from command line
bagfile_name = sys.argv[1]

node = sys.argv[2]

bag = rosbag.Bag(bagfile_name)

out = "/rosout"

for topic, msg, t in bag.read_messages([out]):
    m = message_converter.convert_ros_message_to_dictionary(msg)
    if topic == out:
        if node in m["file"]:
            print(f'{m["msg"]} at {t.to_sec()}')
