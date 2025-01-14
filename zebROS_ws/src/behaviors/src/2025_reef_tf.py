#!/usr/bin/env python3

import rospy
import geometry_msgs
from behavior_actions.msg import AlignToReef2025ActionGoal
import tf2_ros

rospy.init_node('reef_transforms', anonymous=True) #Initializes the node

x_offset = 0.55
y_offset = 0.50
# x_offset = rospy.get_param("reef_x_transform")
# y_offset = rospy.get_param("reef_y_transform")

broadcaster = tf2_ros.StaticTransformBroadcaster() #defines the broadcaster
t = geometry_msgs.msg.TransformStamped()
t.header.stamp = rospy.Time.now()
t.header.frame_id = "base_link"
t.child_frame_id = "pipe"
t.transform.translation.x = x_offset
t.transform.translation.y = y_offset

t.transform.translation.z = 0
t.transform.rotation.x = 0
t.transform.rotation.y = 0
t.transform.rotation.z = 0
t.transform.rotation.w = 1

broadcaster.sendTransform(t)

def callback(msg):
    if msg.goal.pipe == msg.goal.LEFT_PIPE:
        t.transform.translation.y = -y_offset
    else:
        t.transform.translation.y = y_offset

    broadcaster.sendTransform(t)

rospy.Subscriber('/aligner/goal', AlignToReef2025ActionGoal, callback) #subscribes to MatchSpecificData
rospy.spin()