#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float64
from math import hypot

rospy.init_node("this_is_a_node")

x = 0
y = 0

def cb(msg):
    global x, y, pub_speed
    x = msg.twist.linear.x
    y = msg.twist.linear.y
    m = Float64()
    m.data = hypot(x, y)
    pub_speed.publish(m)

sub = rospy.Subscriber("/frcrobot_jetson/swerve_drive_controller/cmd_vel_out", TwistStamped, callback=cb)
pub_speed = rospy.Publisher("/cmd_vel_speed", Float64, queue_size=1)

r = rospy.Rate(100)
rospy.spin()