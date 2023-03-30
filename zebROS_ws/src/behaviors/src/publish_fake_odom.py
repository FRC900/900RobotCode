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

pub = rospy.Publisher("/zed_objdetect/odom", Odometry, queue_size=1)
import time
time.sleep(0.25)

# need to publish zed urdf at the same time
while not rospy.is_shutdown():
    try:
        # from zed objdetect base link to odom
        trans = tfBuffer.lookup_transform("odom", "zed_objdetect_left_camera_frame", rospy.Time(), rospy.Duration(0.1))
        msg = Odometry()
        msg.header.frame_id = "odom"
        msg.child_frame_id = "zed_objdetect_left_camera_frame"
        # msg.pose.pose.position.x = 0.0 #trans.transform.translation.x
        # msg.pose.pose.position.y = 0.0 #trans.transform.translation.y
        # msg.pose.pose.position.z = 0.0 # trans.transform.translation.z
        msg.pose.pose.position.x = trans.transform.translation.x
        msg.pose.pose.position.y = trans.transform.translation.y
        msg.pose.pose.position.z = trans.transform.translation.z
        # msg.pose.pose.orientation.x = 0 #t.quaternion_from_euler(0, 0, -0.00148263)[0] # trans.transform.rotation
        # msg.pose.pose.orientation.y = 0 #t.quaternion_from_euler(0, 0, -0.00148263)[1]
        # msg.pose.pose.orientation.z = 0 #t.quaternion_from_euler(0, 0, -0.00148263)[2]
        # msg.pose.pose.orientation.w = 1 #t.quaternion_from_euler(0, 0, -0.00148263)[3]
        # msg.pose.pose.orientation = trans.transform.rotation

        # only use yaw not pitch or roll (i think the zed reports its pitch & yaw as 0 even though it is tilted down)
        msg.pose.pose.orientation.x = t.quaternion_from_euler(0, 0, t.euler_from_quaternion([trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w])[2])[0]
        msg.pose.pose.orientation.y = t.quaternion_from_euler(0, 0, t.euler_from_quaternion([trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w])[2])[1]
        msg.pose.pose.orientation.z = t.quaternion_from_euler(0, 0, t.euler_from_quaternion([trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w])[2])[2]
        msg.pose.pose.orientation.w = t.quaternion_from_euler(0, 0, t.euler_from_quaternion([trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w])[2])[3]
        pub.publish(msg)

        # from odom to zed objdetect base link
        trans_inv = tfBuffer.lookup_transform("zed_objdetect_left_camera_frame", "odom", rospy.Time(), rospy.Duration(0.1))
        p = PoseStamped()
        p.header = trans_inv.header
        p.pose.position.x = trans_inv.transform.translation.x
        p.pose.position.y = trans_inv.transform.translation.y
        p.pose.position.z = trans_inv.transform.translation.z
        p.pose.orientation = trans_inv.transform.rotation
        p = tf2_geometry_msgs.do_transform_pose(p, tfBuffer.lookup_transform("base_link", "zed_objdetect_left_camera_frame", rospy.Time(), rospy.Duration(0.1)))
        tf_mat = t.concatenate_matrices(t.translation_matrix([p.pose.position.x, p.pose.position.y, p.pose.position.z]), t.quaternion_matrix([p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w]))
        tf_inv = t.inverse_matrix(tf_mat)
        inv_t = t.translation_from_matrix(tf_inv)
        inv_r = t.euler_from_matrix(tf_inv)

        trans_orig = tfBuffer.lookup_transform("odom", "base_link", rospy.Time(), rospy.Duration(0.1))
        print(f"{list(map(lambda a: round(a, 2), inv_t))} vs {trans_orig.transform.translation}")
        print(list(map(lambda a: round(a, 2), inv_r)))
    except Exception as e:
        print(f"error: {e}")
    r.sleep()