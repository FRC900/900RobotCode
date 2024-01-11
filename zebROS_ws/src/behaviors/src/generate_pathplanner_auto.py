#!/usr/bin/env python3
import sys, os

if len(sys.argv) == 2 and sys.argv[1] != "ui":
    print("Usage: generate_pathplanner_auto.py [path_planner_folder_path | \"ui\"] [**path_name**]")
elif len(sys.argv) < 3:
    print("Usage: generate_pathplanner_auto.py [path_planner_folder_path | \"ui\"] [path_name]")

if sys.argv[1] == "ui":
    print("UI not supported yet, use the CLI for now.")
#     current_folder_path = "~"
#     print(f"""\033[2J\033[1;1H ______________________________________________________________________________
# [ FOLDER                                                                       ]
# [ {current_folder_path.ljust(76)[:76]} ]
# [______________________________________________________________________________]""")
    sys.exit(0)

folder = sys.argv[1]
path_name = sys.argv[2]

# Get markers from JSON
import json

# Load JSON
wpipath = json.load(open(f"{folder}/deploy/pathplanner/{path_name}.path"))

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