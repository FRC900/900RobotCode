#!/usr/bin/env python3
# not the actual shooter controller, but probably the fastest way to get running

import rospy
import std_msgs.msg
from math import pi

left_right_percent = 0.0
top_bottom_percent = 0.0
speed = 100.0

def percent_top_bottom_cb(msg):
    global top_bottom_percent
    top_bottom_percent = msg.data
    rospy.loginfo(f"Set top bottom percent to {top_bottom_percent}")

def percent_left_right_cb(msg):
    global left_right_percent
    left_right_percent = msg.data
    rospy.loginfo(f"Set left right percent to {left_right_percent}")

# Convert from native radians per second to RPM
def radiansps_to_rpm(rps):
    return rps / (2. * pi) * 60.


def rpm_to_radiansps(rpm):
    return rpm * (2. * pi) / 60.

# takes in speed in rad/s and percent from 0 to 1 
def speed_cb(msg):
    global left_top_pub, right_top_pub, left_bottom_pub, right_bottom_pub, left_right_percent, top_bottom_percent
    speed = rpm_to_radiansps(msg.data)

    left_right_delta = speed * left_right_percent
    top_bottom_delta = speed * top_bottom_percent

    left_top_speed = speed + left_right_delta + top_bottom_delta
    right_top_speed = speed - left_right_delta + top_bottom_delta
    left_bottom_speed = speed + left_right_delta - top_bottom_delta
    right_bottom_speed = speed - left_right_delta - top_bottom_delta
    left_top_pub.publish(std_msgs.msg.Float64(left_top_speed))
    right_top_pub.publish(std_msgs.msg.Float64(right_top_speed))
    left_bottom_pub.publish(std_msgs.msg.Float64(left_bottom_speed))
    right_bottom_pub.publish(std_msgs.msg.Float64(right_bottom_speed))

    rospy.loginfo(f"\nLT RPM: {radiansps_to_rpm(left_top_speed)}\n RT RPM: {radiansps_to_rpm(right_top_speed)}\n LB RPM: {radiansps_to_rpm(left_bottom_speed)}\n RB RPM: {radiansps_to_rpm(right_bottom_speed)}\n LR%: {left_right_percent}\n TB%: {top_bottom_percent}")

    
rospy.init_node("four_motor_motor_test")
left_top_pub = rospy.Publisher("/frcrobot_jetson/left_top_shooter_voltage_velocity_controller/command", std_msgs.msg.Float64, queue_size=1)
right_top_pub = rospy.Publisher("/frcrobot_jetson/right_top_shooter_voltage_velocity_controller/command", std_msgs.msg.Float64, queue_size=1)
left_bottom_pub = rospy.Publisher("/frcrobot_jetson/left_bottom_shooter_voltage_velocity_controller/command", std_msgs.msg.Float64, queue_size=1)
right_bottom_pub = rospy.Publisher("/frcrobot_jetson/right_bottom_shooter_voltage_velocity_controller/command", std_msgs.msg.Float64, queue_size=1)

rospy.Subscriber("/shooter_speed", std_msgs.msg.Float64, speed_cb)
rospy.Subscriber("/shooter_top_bottom_percent", std_msgs.msg.Float64, percent_top_bottom_cb)
rospy.Subscriber("/shooter_left_right_percent", std_msgs.msg.Float64, percent_left_right_cb)
rospy.spin()