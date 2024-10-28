#!/usr/bin/env python3

import rospy
from frc_utils.match_data_helper import RobotStatusHelper
import time
import re
import os


rospy.init_node("test")
# Example list of files
files = os.listdir("/home/ubuntu/2023RobotCode/zebROS_ws/src/auto_node/src/paths/deploy/choreo")
print(files)

auto_string = "Test4Note"
alliance = "red"
# Define the regex pattern
#pattern = fr"^{auto_string}\d+_(red|blue)$"

pattern = fr"^{auto_string}\d+_({alliance}).csv$"

# Filter the files that match the pattern
matched_files = [file for file in files if re.match(pattern, file)]
matched_files.sort()

print(matched_files)  # Output: ['auto_string1_red', 'auto_string2_blue']

#match_data = RobotStatusHelper()
#time.sleep(1.2)
#while True:
#    rospy.loginfo_throttle(1, f"Enabled {match_data.enabled()}")