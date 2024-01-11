#!/usr/bin/env python3
import sys, os

import tf2_geometry_msgs
from tf import transformations

if len(sys.argv) < 2:
    print("Usage: generate_choreo_auto.py [path_to_chor_file]")
    sys.exit(0)

path_name = sys.argv[1]

# Get markers from JSON
import json

# Load JSON
choreoPath = json.load(open(f"{path_name}"))

paths = choreoPath["paths"]
EPSILON = 0.0001
for path_key in paths:
    print(path_key)
    path = paths[path_key]
    # time,x,y,yaw,angular_velocity,xvel,yvel,waypointIdx
    waypoints = path["waypoints"] + [{'x':0,'y':0}]
    trajectory = path["trajectory"]
    # get inverse transform (we need field -> path)
    new_transform = transformations.concatenate_matrices(transformations.translation_matrix([trajectory[0]['x'],trajectory[0]['y'],0]), transformations.quaternion_matrix(transformations.quaternion_from_euler(0, 0, trajectory[0]['heading'])))
    inversed_transform_mat = transformations.inverse_matrix(new_transform)
    transform = tf2_geometry_msgs.tf2_ros.TransformStamped()
    transform.header.frame_id = "path_frame"
    transform.child_frame_id = "field"
    transform.transform.translation.x = transformations.translation_from_matrix(inversed_transform_mat)[0]
    transform.transform.translation.y = transformations.translation_from_matrix(inversed_transform_mat)[1]
    transform.transform.translation.z = transformations.translation_from_matrix(inversed_transform_mat)[2]
    transform.transform.rotation.x = transformations.quaternion_from_matrix(inversed_transform_mat)[0]
    transform.transform.rotation.y = transformations.quaternion_from_matrix(inversed_transform_mat)[1]
    transform.transform.rotation.z = transformations.quaternion_from_matrix(inversed_transform_mat)[2]
    transform.transform.rotation.w = transformations.quaternion_from_matrix(inversed_transform_mat)[3]
    # init stuff
    csv = ""
    idx = -1
    for t in trajectory:
        # find waypoint index by storing last waypoint hit
        if abs(waypoints[idx+1]['x'] - t['x']) + abs(waypoints[idx+1]['y'] - t['y']) < EPSILON:
            idx += 1
        # add each trajectory waypoint to the csv
        untransformed_pose = tf2_geometry_msgs.PoseStamped()
        untransformed_pose.header.frame_id = "field"
        untransformed_pose.pose.position.x = t['x']
        untransformed_pose.pose.position.y = t['y']
        untransformed_pose.pose.orientation.x = transformations.quaternion_from_euler(0, 0, t['heading'])[0]
        untransformed_pose.pose.orientation.y = transformations.quaternion_from_euler(0, 0, t['heading'])[1]
        untransformed_pose.pose.orientation.z = transformations.quaternion_from_euler(0, 0, t['heading'])[2]
        untransformed_pose.pose.orientation.w = transformations.quaternion_from_euler(0, 0, t['heading'])[3]
        # transform the pose to be relative to first point
        transformed_pose = tf2_geometry_msgs.do_transform_pose(untransformed_pose, transform)
        csv += f"{t['timestamp']},{transformed_pose.pose.position.x},{transformed_pose.pose.position.y},{transformations.euler_from_quaternion([transformed_pose.pose.orientation.x,transformed_pose.pose.orientation.y,transformed_pose.pose.orientation.z,transformed_pose.pose.orientation.w])[2]},{t['angularVelocity']},{t['velocityX']},{t['velocityY']},{idx}\n"
    
    with open(f"/home/ubuntu/2023RobotCode/zebROS_ws/src/behaviors/path/{path_key}_csv.csv", "w") as csv_file:
        print(csv)
        csv_file.write(csv)
    
sys.exit(0)

# Parse markers
markers = sorted(list(map(lambda m: (m["position"], m["names"][0]), wpipath["markers"])), key=lambda m: m[0], reverse=True)

# Function to transform time -> waypoint ID

def markerID(t):
    # Markers must be in time-descending order

    for i in range(len(markers)):
        if t >= markers[i][0] - 0.05:
            return i+1
    
    # If the time was not close to any marker, return 0
    return 0

formatted_csv = ""

waypoints = 0

# Delete any commented CSV lines
with open(f"{folder}/deploy/pathplanner/generatedCSV/{path_name}.csv", "r") as csv:
    for line in csv.readlines():
        if line[0] == '#':
            continue # comment
        waypoints += 1
        # Add the waypoint ID to the end (the index of the next marker)
        formatted_csv += line[:-1] + "," + str(markerID(float(line.split(",")[0]))) + "\n"

with open(f"/home/ubuntu/2023RobotCode/zebROS_ws/src/behaviors/path/{path_name}_csv.csv", "w") as csv:
    csv.write(formatted_csv)

print()
print("---------------------------- GENERATED SUCCESSFULLY ----------------------------")
print()
print("Event list:", markers)
print("# Waypoints:", waypoints)
print(f"Saved to /home/ubuntu/2023RobotCode/zebROS_ws/src/behaviors/path/{path_name}_csv.csv")
print()
print("------------------------------ TEMPLATE YAML CODE ------------------------------\n")
yaml = f"""auto_mode_900: ["{path_name}_csv"]

{path_name}_csv:
  waypoint_actions: \x7b""" # \x7b = "{", didn't want to trigger f-string behavior
for i in range(len(markers)):
    yaml += f"{markers[i][1]}: {i+1}, "
yaml = yaml[:-2] # trim trailing ", "
yaml += "}"
yaml += """
  type: "path"
  goal:
    points:
      - [0, 0, 0]
"""
print(yaml)
print()
print("-"*80)
print()