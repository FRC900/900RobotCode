# Take in a rosbag as a command line argument
# Analyze the rosbag, and for each time an apriltag detection is received,
# print out the velocity (/frcrobot_jetson/swerve_drive_controller/cmd_vel_out)

import rosbag
import sys
import os

bagfile_name = sys.argv[1]

bag = rosbag.Bag(bagfile_name)

last_vel = None
last_tf = None

for topic, msg, t in bag.read_messages(topics=["/frcrobot_jetson/swerve_drive_controller/cmd_vel_out", "/apriltag_zedx_back/apriltag_detection/tags", "/apriltag_zedx_front/apriltag_detection/tags"]):
    if topic == "/apriltag_zedx_back/apriltag_detection/tags" or topic == "/apriltag_zedx_front/apriltag_detection/tags":
        if len(msg.apriltags) > 0:
            print(f"{t.to_sec()},{(last_vel.twist.linear.x**2 + last_vel.twist.linear.y**2)**0.5},{last_vel.twist.angular.z}")
    elif topic == "/frcrobot_jetson/swerve_drive_controller/cmd_vel_out":
        last_vel = msg

