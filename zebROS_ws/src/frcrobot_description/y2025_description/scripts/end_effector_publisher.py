#!/usr/bin/env python3

import rospy
from talon_state_msgs.msg import TalonFXProState
import tf2_ros
from geometry_msgs.msg import TransformStamped

# rosrun tf2_ros static_transform_publisher 0 0 0.5 0 0 0 base_link end_effector_link

rospy.init_node("end_effector_publisher")

br = tf2_ros.TransformBroadcaster()
t = TransformStamped()