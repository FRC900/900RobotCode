#!/usr/bin/env python3
# generates red and blue versions of a path
# updated to work with choreo 2025, in which the ENTIRE path is in one .traj file, stop points are define in the path
# need to find where stop points are in the .traj file
# directory structure is auto_node/paths/*.traj

import sys, os, math, json, datetime
import tf2_geometry_msgs
from tf import transformations

print("SHOULD BE RUN FROM CMAKE!!")

os.chdir("/home/ubuntu/2023RobotCode/zebROS_ws/src/auto_node/scripts")
with open("path_info.txt", "w+") as f:
    f.write(f"Last paths regenerated at {datetime.datetime.now()}\n ")
    os.chdir("../paths")
    f.write(f"Using {os.getcwd()} dir\n")

    files = os.listdir(".")
    # dont want to run on a csv
    files = [file for file in files if file.endswith('.traj')]

    f.write(str(files))




# assume start of path is a stop point (obviously) so 2 extra stop points is 3 path segments or 6 csvs to read
# current marker is now which section of the split you are on, can obviously change since waypoints are accessible
for file in files:
    try:
        trajectory = json.load(open(file, "r"))["trajectory"]
    except Exception as e:
        print(f"Failed on {file} with exception {e}")

    samples = trajectory["samples"]
    splits = trajectory["splits"] 
    splits.append(0) # include the start and end as a points to start/stop a path
    splits.append(len(samples)-1) # like not perfect but will make sure all the segments of the path are generated
    print(f"Splits {splits} samples ")
    print(samples)
    current_segment = 0 
    for idx, t in enumerate(samples):
        print(idx, t)
        if idx in splits:
            print(f"Found {idx} in splits \n\n\n")
            if current_segment > 0:
                with open(f"{file.replace('.traj', '')}{current_segment-1}_blue.csv", "w") as blue_file:
                    print(csv_blue)
                    blue_file.write(csv_blue)

                with open(f"{file.replace('.traj', '')}{current_segment-1}_red.csv", "w") as red_file:
                    print(csv_red)
                    red_file.write(csv_red)
            # init stuff
            csv_blue = ""
            csv_red = ""
            current_segment += 1
            time_offset = t["t"]

        # add each trajectory waypoint to the csv
        pose = tf2_geometry_msgs.PoseStamped()
        pose.header.frame_id = "field"
        pose.pose.position.x = 16.54 - t['x']
        pose.pose.position.y = t['y']

        csv_red += f"{t['t']-time_offset},{16.54 - t['x']},{t['y']},{math.pi - t['heading']},{-t['omega']},{-t['vx']},{t['vy']},{current_segment}\n"
        csv_blue += f"{t['t']-time_offset},{t['x']},{t['y']},{t['heading']},{t['omega']},{t['vx']},{t['vy']},{current_segment}\n"

''' 
if current_segment > 0:
    with open(f"{file.replace('.traj', '')}{current_segment-1}_blue.csv", "w") as blue_file:
        print(csv_blue)
        blue_file.write(csv_blue)

    with open(f"{file.replace('.traj', '')}{current_segment-1}_red.csv", "w") as red_file:
        print(csv_red)
        red_file.write(csv_red)'''