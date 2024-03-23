#!/usr/bin/env python3

import rosbag
import sys

last_tf_stamp = None
last_relocalize_time = None
last_cmd_vel = None
last_limit_state = None
last_cmd_vel_at_tf = None

for topic, msg, t in rosbag.Bag(sys.argv[1]).read_messages(["/last_relocalize", "/tf", "/frcrobot_rio/joint_states", "/frcrobot_jetson/swerve_drive_controller/odom"]):
    if topic == "/last_relocalize":
        last_relocalize_time = msg.stamp
    elif topic == "/tf":
        for transform in msg.transforms:
            if transform.header.frame_id == "map":
                last_tf_stamp = transform.header.stamp
                last_cmd_vel_at_tf = last_cmd_vel
    elif topic == "/frcrobot_rio/joint_states":
        limit_state = msg.position[1]
        if last_limit_state and not limit_state:
            try:
                print(f"{t.to_sec()},{last_relocalize_time.to_sec()},{last_tf_stamp.to_sec()},{last_cmd_vel_at_tf.linear.x},{last_cmd_vel_at_tf.linear.y},{last_cmd_vel_at_tf.angular.z}")
            except: pass
        last_limit_state = limit_state
    elif topic == "/frcrobot_jetson/swerve_drive_controller/odom":
        last_cmd_vel = msg.twist.twist