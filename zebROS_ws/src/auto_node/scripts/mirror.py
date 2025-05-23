#!/usr/bin/env python3
import json, sys

with open("2025_2_Mid_Coral.traj") as f:
    data = json.load(f)

new_waypoints = []

# 8.050 / 2
for waypoint in data["params"]["waypoints"]:
    new_waypoint = waypoint
    new_waypoint['y']['exp'] = f"{8.050 - float(waypoint['y']['exp'][:-2])} m"
    new_waypoint['y']['val'] = 8.050 - float(waypoint['y']['exp'][:-2])
    new_waypoint['heading']['exp'] = f"{-float(waypoint['heading']['exp'][:-4])} deg"
    new_waypoint['heading']['val'] = -float(waypoint['heading']['exp'][:-4])
    print(new_waypoint)
    new_waypoints.append(waypoint)

data["params"]["waypoints"] = new_waypoints

with open("2025_2_Mid_Coral_NonProcessor.traj", "w") as f:
    json.dump(data, f)