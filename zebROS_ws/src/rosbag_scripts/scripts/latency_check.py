#!/usr/bin/env python

import rosbag
import math

ENABLE_TOPIC = '/frcrobot_rio/match_data'
CMD_VEL_TOPIC = '/frcrobot_jetson/swerve_drive_controller/cmd_vel_out'
VEL_THRESHOLD = 0.01  # Threshold to consider velocity as non-zero

def get_enable_time(bag):
    for topic, msg, t in bag.read_messages(topics=[ENABLE_TOPIC]):
        try:
            if msg.Enabled:  # Adjust this if the actual field name is different
                return t.to_sec()
        except AttributeError:
            print("Message from /frcrobot_rio/match_data does not have 'enabled' field.")
            continue
    return None

def get_first_nonzero_cmd_vel_time(bag):
    for topic, msg, t in bag.read_messages(topics=[CMD_VEL_TOPIC]):
        try:
            vel = math.hypot(msg.twist.linear.x, msg.twist.linear.y)
            if vel > VEL_THRESHOLD:
                return t.to_sec()
        except AttributeError:
            print("cmd_vel message missing linear.x or linear.y.")
            continue
    return None

def main(bagfile):
    bag = rosbag.Bag(bagfile)

    enable_time = get_enable_time(bag)
    if enable_time is None:
        print("Enable signal not found in bag.")
        return

    cmd_vel_time = get_first_nonzero_cmd_vel_time(bag)
    if cmd_vel_time is None:
        print("Non-zero cmd_vel signal not found in bag.")
        return

    latency = cmd_vel_time - enable_time
    print(f"Latency between enable and first non-zero cmd_vel: {latency:.3f} seconds")

    bag.close()

main("rosbags/19700101_000642_NCCMP_Q12.bag")
