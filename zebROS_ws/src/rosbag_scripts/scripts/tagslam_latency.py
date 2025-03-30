#!/usr/bin/env python3
import rosbag
import sys
import matplotlib.pyplot as plt
import math

times = []
latencies = []
tags_9_0 = []
tags_10_0 = []
tags_9_1 = []
tags_10_1 = []
vels = []
ang_vels = []
for topic, msg, t in rosbag.Bag(sys.argv[1]).read_messages(["/tagslam/odom/body_frc_robot", "/apriltag_detection_ov2311_10_9_0_9_video0/tags", "/apriltag_detection_ov2311_10_9_0_9_video1/tags", "/apriltag_detection_ov2311_10_9_0_10_video0/tags", "/apriltag_detection_ov2311_10_9_0_10_video1/tags", "/frcrobot_jetson/swerve_drive_controller/cmd_vel_out"]):
    if topic == "/tagslam/odom/body_frc_robot":
        times.append(t.to_sec())
        latencies.append(t.to_sec() - msg.header.stamp.to_sec())
    if topic == "/apriltag_detection_ov2311_10_9_0_9_video0/tags":
        tags_9_0.append((t.to_sec(), len(msg.apriltags)))
    if topic == "/apriltag_detection_ov2311_10_9_0_10_video0/tags":
        tags_10_0.append((t.to_sec(), len(msg.apriltags)))
    if topic == "/apriltag_detection_ov2311_10_9_0_9_video1/tags":
        tags_9_1.append((t.to_sec(), len(msg.apriltags)))
    if topic == "/apriltag_detection_ov2311_10_9_0_10_video1/tags":
        tags_10_1.append((t.to_sec(), len(msg.apriltags)))
    if topic == "/frcrobot_jetson/swerve_drive_controller/cmd_vel_out":
        vels.append((t.to_sec(), math.hypot(msg.twist.linear.x, msg.twist.linear.y)))
        ang_vels.append((t.to_sec(), abs(msg.twist.angular.z)))

plt.plot(times, latencies, '-', color="black", label="latencies")
# plt.plot(*zip(*tags_9_0), '-', color="green", label="tags9_0")
# plt.plot(*zip(*tags_9_1), '-', color="blue", label="tags9_1")
# plt.plot(*zip(*tags_10_0), '-', color="red", label="tags10_0")
# plt.plot(*zip(*tags_10_1), '-', color="yellow", label="tags10_1")
# plt.plot(*zip(*vels), '-', color="pink", label="vel")
# plt.plot(*zip(*ang_vels), '-', color="purple", label="ang_vel")
plt.legend()
plt.show()

plt.hist(latencies, bins=100)
plt.show()