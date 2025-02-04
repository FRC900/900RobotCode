#!/usr/bin/env python3

import rospy
import geometry_msgs
import tf2_ros
import time
import copy

rospy.init_node('reef_transforms', anonymous=True) #Initializes the node

time.sleep(1)

# x_offset = 0.55
# y_offset = -0.5
x_offset = rospy.get_param("reef_x_transform")
y_offset = rospy.get_param("reef_y_transform")

broadcaster = tf2_ros.StaticTransformBroadcaster() #defines the broadcaster
t = geometry_msgs.msg.TransformStamped()
t.header.frame_id = "base_link"
t.transform.translation.x = x_offset

t.transform.translation.z = 0
t.transform.rotation.x = 0
t.transform.rotation.y = 0
t.transform.rotation.z = 0
t.transform.rotation.w = 1

t.header.stamp = rospy.Time.now()
t.child_frame_id = "left_pipe"
t.transform.translation.y = y_offset # left
broadcaster.sendTransform(t)

t2 = copy.deepcopy(t)

t2.header.stamp = rospy.Time.now()
t2.child_frame_id = "right_pipe"
t2.transform.translation.y = -y_offset # right
broadcaster.sendTransform(t2)

r = rospy.Rate(1)
while not rospy.is_shutdown():
    broadcaster.sendTransform(t)
    broadcaster.sendTransform(t2)
    r.sleep()

rospy.spin()