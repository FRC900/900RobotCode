#!/usr/bin/env python3
import rosbag
import sys
import matplotlib.pyplot as plt
import math
import angles
from tf.transformations import euler_from_quaternion

deltas = []
enableds = []
zero_times = []
latest_imu_yaw = 0
for topic, msg, t in rosbag.Bag(sys.argv[1]).read_messages(["/tagslam/odom/body_frc_robot", "/imu/zeroed_imu", "/frcrobot_rio/match_data", "/rosout"]):
    if topic == "/imu/zeroed_imu":
        latest_imu_yaw = euler_from_quaternion([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])[2]
    if topic == "/tagslam/odom/body_frc_robot":
        latest_tagslam_yaw = euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])[2]
        deltas.append((t.to_sec(), angles.shortest_angular_distance(latest_imu_yaw, latest_tagslam_yaw)))
    if topic == "/frcrobot_rio/match_data":
        enableds.append((t.to_sec(), int(msg.Enabled)))
    if topic == "/rosout":
        if msg.name == "/imu/imu_zero":
            zero_times.append(t.to_sec())

ax = plt.gca()
# ax.set_xlim([xmin, xmax])
ax.set_ylim([-0.2, 0.2])
plt.plot(*zip(*deltas), '-', color="black", label="deltas")
plt.plot(*zip(*enableds), '-', color="green", label="enableds")

for xc in zero_times:
    if xc == zero_times[0]:
        plt.axvline(x=xc, color='purple', linestyle='-', alpha=0.5, label="Zeroed")
    else:
        plt.axvline(x=xc, color='purple', linestyle='-', alpha=0.5)

plt.legend()
plt.savefig(sys.argv[1]+".png", dpi=300)
plt.show()