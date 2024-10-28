#!/usr/bin/env python3

# generates red and blue versions of a path
import sys, os, math, json

import tf2_geometry_msgs
from tf import transformations

if len(sys.argv) < 2:
    print("Usage: generate_choreo_auto_map.py [deploy/choreo]")
    sys.exit(0)

path_name = sys.argv[1]
os.chdir(path_name)
print(f"Using {path_name} dir")

files = os.listdir(".")

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

    with open(f"/home/ubuntu/900RobotCode/zebROS_ws/src/behaviors/path/{file.replace('.traj', '').replace('.', '')}_blue_csv.csv", "w") as blue_file:
        print(csv_blue)
        blue_file.write(csv_blue)
    
    with open(f"/home/ubuntu/900RobotCode/zebROS_ws/src/behaviors/path/{file.replace('.traj', '').replace('.', '')}_red_csv.csv", "w") as red_file:
        print(csv_red)
        red_file.write(csv_red)