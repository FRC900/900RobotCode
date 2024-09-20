#!/usr/bin/env python3

import rospy
#from frc_utils import match_data_helper 
from frc_utils.match_data_helper import RobotStatusHelper

import inspect
import sys
import time
print(sys.modules.keys())
rospy.init_node("test")

match_data = RobotStatusHelper()
time.sleep(1.2)
print("most recent version no /home/ubuntu/zebROS_ws/src/frc_utils/src")
while True:
    rospy.loginfo_throttle(1, f"Enabled {match_data.enabled()}")