#!/usr/bin/env python3

import rospy
from frc_utils.match_data_helper import RobotStatusHelper
import time
rospy.init_node("test")

match_data = RobotStatusHelper()
time.sleep(1.2)
while True:
    rospy.loginfo_throttle(1, f"Enabled {match_data.enabled()}")