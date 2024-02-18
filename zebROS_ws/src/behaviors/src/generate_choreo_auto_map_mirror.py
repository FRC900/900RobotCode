#!/usr/bin/env python3
import sys, os, math, json

import tf2_geometry_msgs
from tf import transformations

if len(sys.argv) < 2:
    print("Usage: generate_choreo_auto_map_red.py [deploy/choreo]")
    sys.exit(0)

path_name = sys.argv[1]
os.chdir(path_name)
print(f"Using {path_name} dir")

files = os.listdir(".")

EPSILON = 0.0001
for file in files:
    try:
        trajectory = json.load(open(file, "r"))["samples"]
    except Exception as e:
        print(f"Failed on {file} with exception {e}")
    # init stuff
    csv = ""
    idx = -1
    for t in trajectory:
        # add each trajectory waypoint to the csv
        pose = tf2_geometry_msgs.PoseStamped()
        pose.header.frame_id = "field"
        pose.pose.position.x = 16.54 - t['x']
        pose.pose.position.y = t['y']
        pose.pose.orientation.x = transformations.quaternion_from_euler(0, 0, t['heading'])[0]
        pose.pose.orientation.y = transformations.quaternion_from_euler(0, 0, t['heading'])[1]
        pose.pose.orientation.z = transformations.quaternion_from_euler(0, 0, t['heading'])[2]
        pose.pose.orientation.w = transformations.quaternion_from_euler(0, 0, t['heading'])[3]

        csv += f"{t['timestamp']},{pose.pose.position.x},{pose.pose.position.y},{math.pi - transformations.euler_from_quaternion([pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w])[2]},{t['angularVelocity']},{t['velocityX']},{t['velocityY']},0\n"
    with open(f"/home/ubuntu/2023RobotCode/zebROS_ws/src/behaviors/path/map_rel_{file.replace('.traj', '').replace('.', '')}_red_csv.csv", "w") as csv_file:
        print(csv)
        csv_file.write(csv)