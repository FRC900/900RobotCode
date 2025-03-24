#!/usr/bin/env python3
import rospy
import rosbag
import tf2_ros
from tf2_geometry_msgs import do_transform_pose
from geometry_msgs.msg import PoseStamped, TransformStamped
from path_follower_msgs.msg import PathActionFeedback, PathActionResult, PathActionGoal

from scipy.spatial.transform import Rotation as R
import numpy as np

import math

rospy.init_node("tf_finder", anonymous=True)

tf_buffer = tf2_ros.Buffer()
tf_listener = tf2_ros.TransformListener(tf_buffer)

bag_path = "/home/ubuntu/900RobotCode/bagfiles/auto_testing_march24/auto_test_fourth_kevin_video_2025-03-24-18-25-54.bag"

bag = rosbag.Bag(bag_path)

path_following_running = False

csv = """time,tagslam_latency_ms,distance_between_frcrobot_and_baselink,is_path_following_running\n"""

for topic, msg_, t in bag.read_messages(topics=['/tf', '/tf_static', '/path_follower/path_follower_server/result', '/path_follower/path_follower_server/goal', '/frcrobot_rio/match_data']):
    if topic == "/tf" or topic == "/tf_static":
        msg: TransformStamped = msg_
        for transform in msg.transforms:
            try:
                if topic == '/tf_static':
                    tf_buffer.set_transform_static(transform, "default_authority")
                elif topic == '/tf':
                    tf_buffer.set_transform(transform, "default_authority")
            except (tf2_ros.ExtrapolationException, tf2_ros.ConnectivityException, tf2_ros.LookupException) as e:
                rospy.logwarn(f"Error adding transform: {e}")
    
    if topic == '/path_follower/path_follower_server/goal':
        print("******************************PATH FOLLOWING STARTED")
        path_following_running = True
    
    if topic == '/path_follower/path_follower_server/result':
        path_following_running = False
        print("******************************PATH FOLLOWING STOPPED")

    try:
        # if path_following_running:
        tagslam_latest: TransformStamped = tf_buffer.lookup_transform("map", "frc_robot", rospy.Time(0))
        base_link_latest: TransformStamped = tf_buffer.lookup_transform("map", "base_link", rospy.Time(0))
        latency = base_link_latest.header.stamp.to_sec() - tagslam_latest.header.stamp.to_sec()
        pos_difference = math.hypot(tagslam_latest.transform.translation.x - base_link_latest.transform.translation.x, tagslam_latest.transform.translation.y - base_link_latest.transform.translation.y)
        csv += f"{t.to_sec()},{latency*1000},{pos_difference},{path_following_running}\n"
    except:
        pass

bag.close()

with open(f"{bag_path}.csv", "w") as f:
    f.write(csv)
