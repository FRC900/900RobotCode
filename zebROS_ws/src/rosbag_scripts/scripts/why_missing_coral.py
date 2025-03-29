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

import sys

import matplotlib.pyplot as plt

from tf.transformations import euler_from_quaternion
from angles import shortest_angular_distance

rospy.init_node("tf_finder", anonymous=True)

tf_buffer = tf2_ros.Buffer()
tf_listener = tf2_ros.TransformListener(tf_buffer)

bag_path = sys.argv[1]

print(f"\nOpening {bag_path}:\n")

bag = rosbag.Bag(bag_path)

path_following_running = False
has_coral = False

times = []
latencies = []
pos_diffs = []
path_runnings = []
has_corals = []

full_optimizations = []
dropped_tags = []

vels = []
ang_vels = []
ang_diffs = []

csv = """time,tagslam_latency_ms,distance_between_frcrobot_and_baselink,is_path_following_running,has_coral\n"""

for topic, msg, t in bag.read_messages(topics=['/tf', '/tf_static', '/path_follower/path_follower_server/result', '/path_follower/path_follower_server/goal', '/frcrobot_jetson/swerve_drive_controller/odom', '/frcrobot_rio/joint_states', '/rosout', '/frcrobot_jetson/swerve_drive_controller/cmd_vel_out']):
    if topic == "/tf" or topic == "/tf_static":
        for transform in msg.transforms:
            try:
                if topic == '/tf_static':
                    tf_buffer.set_transform_static(transform, "default_authority")
                elif topic == '/tf':
                    tf_buffer.set_transform(transform, "default_authority")
            except (tf2_ros.ExtrapolationException, tf2_ros.ConnectivityException, tf2_ros.LookupException) as e:
                rospy.logwarn(f"Error adding transform: {e}")
    
    try:
        tagslam_latest: TransformStamped = tf_buffer.lookup_transform("map", "frc_robot", rospy.Time(0))
        base_link_latest: TransformStamped = tf_buffer.lookup_transform("map", "base_link", rospy.Time(0))
        latency = base_link_latest.header.stamp.to_sec() - tagslam_latest.header.stamp.to_sec()
        pos_difference = math.hypot(tagslam_latest.transform.translation.x - base_link_latest.transform.translation.x, tagslam_latest.transform.translation.y - base_link_latest.transform.translation.y)
        tagslam_yaw = euler_from_quaternion([tagslam_latest.transform.rotation.x, tagslam_latest.transform.rotation.y, tagslam_latest.transform.rotation.z, tagslam_latest.transform.rotation.w])[2]
        base_link_yaw = euler_from_quaternion([base_link_latest.transform.rotation.x, base_link_latest.transform.rotation.y, base_link_latest.transform.rotation.z, base_link_latest.transform.rotation.w])[2]
        ang_difference = abs(shortest_angular_distance(tagslam_yaw, base_link_yaw)) * 180/math.pi
    except Exception as e:
        print(e)
        pass

    if topic == '/path_follower/path_follower_server/goal':
        print("******************************PATH FOLLOWING STARTED")
        path_following_running = True
    
    if topic == '/path_follower/path_follower_server/result':
        path_following_running = False
        print(f"******************************PATH FOLLOWING STOPPED, DELTA POS = {pos_difference}, LATENCY = {latency}")
    
    if topic == '/frcrobot_rio/joint_states':
        if "roller_limit_switch" in msg.name:
            prev_has_coral = has_coral
            has_coral = msg.position[msg.name.index("roller_limit_switch")]
            if has_coral != prev_has_coral:
                print(f"Coral state changed, now has_coral={has_coral}")

    if topic == '/frcrobot_jetson/swerve_drive_controller/odom':
        # using as 250 Hz clock
        try:
            csv += f"{t.to_sec()},{latency*1000},{pos_difference},{int(path_following_running)},{int(has_coral)}\n"
            times.append(t.to_sec())
            latencies.append(latency)
            pos_diffs.append(pos_difference)
            path_runnings.append(int(path_following_running))
            has_corals.append(int(has_coral))
            vels.append(latest_vel)
            ang_vels.append(latest_ang_vel)
            ang_diffs.append(ang_difference)
        except:
            pass

    if topic == '/rosout':
        if "running full optimization" in msg.msg:
            full_optimizations.append(t.to_sec())
        if "drop tag with low viewing angle" in msg.msg:
            dropped_tags.append(t.to_sec())
    
    if topic == "/frcrobot_jetson/swerve_drive_controller/cmd_vel_out":
        latest_vel = math.hypot(msg.twist.linear.x, msg.twist.linear.y)
        latest_ang_vel = abs(msg.twist.angular.z)

bag.close()

with open(f"{bag_path}.csv", "w") as f:
    f.write(csv)

plt.plot(times, pos_diffs, color='blue', linestyle='-', label='Pos delta (m)')
plt.plot(times, latencies, color='green', linestyle='-', label='Latency (s)')
plt.plot(times, path_runnings, color='red', linestyle='-', label='Path following running')
plt.plot(times, has_corals, color='yellow', linestyle='-', label='Has coral')
plt.plot(times, vels, '-', color="pink", label="vel")
plt.plot(times, ang_vels, '-', color="purple", label="ang_vel")
plt.plot(times, ang_diffs, '-', color="orange", label="ang_diffs")

# for xc in dropped_tags:
#     if xc == dropped_tags[0]:
#         plt.axvline(x=xc, color='purple', linestyle='-', alpha=0.5, label="Dropped tags due to low viewing angle")
#     else:
#         plt.axvline(x=xc, color='purple', linestyle='-', alpha=0.5)

plt.legend()
plt.show()