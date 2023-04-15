import os
import rospy
import rosbag
from rospy_message_converter import message_converter
import sys

#Bagfile name from command line
bagfile_name = sys.argv[1]

bag = rosbag.Bag(bagfile_name)

talon = '/frcrobot_jetson/talon_states'
auto = False

last_match_time = 0

with open("fourbar.csv", "w") as f:
    f.write("time,position,setpoint,motor_output\n")
    for topic, msg, t in bag.read_messages([talon]):
        m = message_converter.convert_ros_message_to_dictionary(msg)
        f.write(f"{t.to_sec()},{m['position'][8]},{m['active_trajectory_position'][8]},{m['motor_output_percent'][8]}\n")
