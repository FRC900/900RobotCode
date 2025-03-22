#!/usr/bin/env python3
import rospy
import rosbag
import tf2_ros
from tf2_geometry_msgs import do_transform_pose
from geometry_msgs.msg import PoseStamped, TransformStamped

from scipy.spatial.transform import Rotation as R
import numpy as np

rospy.init_node("tf_finder", anonymous=True)

tf_buffer = tf2_ros.Buffer()
tf_listener = tf2_ros.TransformListener(tf_buffer)

bag = rosbag.Bag('/home/ubuntu/.ros/out.bag')

for topic, msg_, t in bag.read_messages(topics=['/tf', '/tf_static']):
    msg: TransformStamped = msg_
    for transform in msg.transforms:
        try:
            if topic == '/tf_static':
                tf_buffer.set_static_transform(transform, "default_authority")
            else:
                tf_buffer.set_transform(transform, "default_authority")
        except (tf2_ros.ExtrapolationException, tf2_ros.ConnectivityException, tf2_ros.LookupException) as e:
            rospy.logwarn(f"Error adding transform: {e}")

bag.close()

for c in range(4):
    try:
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = f"cam{c}"
        pose_stamped.header.stamp = rospy.Time(0)
        pose_stamped.pose.orientation.w = 1.0

        transform_stamped = tf_buffer.lookup_transform("map", f"cam{c}", rospy.Time(0))
        transformed_pose: PoseStamped = do_transform_pose(pose_stamped, transform_stamped)

        r = R.from_quat([transformed_pose.pose.orientation.x,transformed_pose.pose.orientation.y,transformed_pose.pose.orientation.z,transformed_pose.pose.orientation.w])
        axis_angle = r.as_rotvec()

        print(f"""
cam{c}:
  pose:
    position:
      x: {transformed_pose.pose.position.x}
      y: {transformed_pose.pose.position.y}
      z: {transformed_pose.pose.position.z}
    rotation:
      x: {axis_angle[0]}
      y: {axis_angle[1]}
      z: {axis_angle[2]}
              """)

        # rospy.loginfo(f"Transformed pose: {transformed_pose}")
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        rospy.logwarn(f"Error looking up transform: {e}")

pose_stamped = PoseStamped()
pose_stamped.header.frame_id = f"frc_robot"
pose_stamped.header.stamp = rospy.Time(0)
pose_stamped.pose.orientation.w = 1.0

transform_stamped = tf_buffer.lookup_transform("map", f"frc_robot", rospy.Time(0))
transformed_pose: PoseStamped = do_transform_pose(pose_stamped, transform_stamped)

print(transformed_pose)

pose_stamped = PoseStamped()
pose_stamped.header.frame_id = f"cam0"
pose_stamped.header.stamp = rospy.Time(0)
pose_stamped.pose.orientation.w = 1.0

transform_stamped = tf_buffer.lookup_transform("cam1", f"cam0", rospy.Time(0))
transformed_pose: PoseStamped = do_transform_pose(pose_stamped, transform_stamped)

print(transformed_pose)