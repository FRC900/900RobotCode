#!/usr/bin/env python3

# generates red and blue versions of a path
import sys, os, math, json, datetime

import tf2_geometry_msgs
from tf import transformations

print("RUN FROM CMAKE!!")
os.chdir("/home/ubuntu/.2023RobotCode.readonly/zebROS_ws/src/auto_node/scripts")
with open("path_info.txt", "w+") as f:
    f.write(f"Last paths regenerated at {datetime.datetime.now()}\n ")
    os.chdir("../src/paths/deploy/choreo")
    f.write(f"Using {os.getcwd()} dir\n")

    files = os.listdir(".")
    # dont want to run on a csv
    files = [file for file in files if file.endswith('.traj')]

    f.write(str(files))

EPSILON = 0.0001
for file in files:
    try:
        trajectory = json.load(open(file, "r"))["samples"]
        markers = list(map(lambda m: m["timestamp"], json.load(open(file, "r"))["eventMarkers"]))
    except Exception as e:
        print(f"Failed on {file} with exception {e}")
    # init stuff
    csv_blue = ""
    csv_red = ""
    idx = -1
    current_marker = 0
    for t in trajectory:
        # add each trajectory waypoint to the csv
        pose = tf2_geometry_msgs.PoseStamped()
        pose.header.frame_id = "field"
        pose.pose.position.x = 16.54 - t['x']
        pose.pose.position.y = t['y']

        if len(markers) > 0 and current_marker < len(markers):
            if markers[current_marker] < t["timestamp"]:
                current_marker += 1

        csv_red += f"{t['timestamp']},{16.54 - t['x']},{t['y']},{math.pi - t['heading']},{-t['angularVelocity']},{-t['velocityX']},{t['velocityY']},{current_marker}\n"
        csv_blue += f"{t['timestamp']},{t['x']},{t['y']},{t['heading']},{t['angularVelocity']},{t['velocityX']},{t['velocityY']},{current_marker}\n"

    with open(f"{file.replace('.traj', '').replace('.', '')}_blue.csv", "w") as blue_file:
        print(csv_blue)
        blue_file.write(csv_blue)
    
    with open(f"{file.replace('.traj', '').replace('.', '')}_red.csv", "w") as red_file:
        print(csv_red)
        red_file.write(csv_red)