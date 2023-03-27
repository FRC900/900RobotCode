#!/usr/bin/env python3
import rospy
import tf2_ros
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import tf2_geometry_msgs

from tf import transformations as t
import numpy as np

rospy.init_node('fake_odom_pub')

tfBuffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tfBuffer)

r = rospy.Rate(100)

pub = rospy.Publisher("/fake_odometry", Odometry, queue_size=1)
import time
time.sleep(0.25)

# need to publish zed urdf at the same time
while not rospy.is_shutdown():
    # from zed objdetect base link to odom
    trans = tfBuffer.lookup_transform("odom", "zed_objdetect_base_link", rospy.Time(), rospy.Duration(0.1))
    msg = Odometry()
    msg.header.frame_id = "odom"
    msg.child_frame_id = "zed_objdetect_base_link"
    msg.pose.pose.position.x = trans.transform.translation.x
    msg.pose.pose.position.y = trans.transform.translation.y
    msg.pose.pose.position.z = trans.transform.translation.z
    msg.pose.pose.orientation = trans.transform.rotation
    pub.publish(msg)

    # from odom to zed objdetect base link
    trans_inv = tfBuffer.lookup_transform("zed_objdetect_base_link", "odom", rospy.Time(), rospy.Duration(0.1))
    p = PoseStamped()
    p.header = trans_inv.header
    p.pose.position.x = trans_inv.transform.translation.x
    p.pose.position.y = trans_inv.transform.translation.y
    p.pose.position.z = trans_inv.transform.translation.z
    p.pose.orientation = trans_inv.transform.rotation
    p = tf2_geometry_msgs.do_transform_pose(p, tfBuffer.lookup_transform("base_link", "zed_objdetect_base_link", rospy.Time(), rospy.Duration(0.1)))
    tf_mat = t.concatenate_matrices(t.translation_matrix([p.pose.position.x, p.pose.position.y, p.pose.position.z]), t.quaternion_matrix([p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w]))
    tf_inv = t.inverse_matrix(tf_mat)
    inv_t = t.translation_from_matrix(tf_inv)
    inv_r = t.euler_from_matrix(tf_inv)

    trans_orig = tfBuffer.lookup_transform("odom", "base_link", rospy.Time(), rospy.Duration(0.1))
    print(f"{list(map(lambda a: round(a, 2), inv_t))} vs {trans_orig.transform.translation}")
    print(list(map(lambda a: round(a, 2), inv_r)))
    r.sleep()