import os
import rospy
import rosbag
from rospy_message_converter import message_converter
import sys

#Bagfile name from command line
bagfile_name = sys.argv[1]

bag = rosbag.Bag(bagfile_name)

out = "/rosout"
match = "/frcrobot_rio/match_data"
auto = False

last_match_time = 0

crc_errors = []
crc_start = 0
crc_auto = False
for topic, msg, t in bag.read_messages([out, match]):
    m = message_converter.convert_ros_message_to_dictionary(msg)
    if topic == out:
        if "CRC error" in m["msg"] or "AutoPlaceState" in m["msg"] or "IMU too far" in m["msg"]:
            if crc_start == 0:
                crc_start = last_match_time
                crc_auto = auto
            crc_errors.append(last_match_time + (135 if auto and last_match_time != 135.0 else 0))
    elif topic == match:
        last_match_time = m["matchTimeRemaining"]
        auto = m["Autonomous"]

print(f"{bagfile_name}: ", end="")
print(f"{len(crc_errors)} errors @ {crc_errors}")
print() # newline
#print(f"{len(crc_errors)} CRC errors. Started @ {crc_start} {'auto' if crc_auto else 'teleop'}. Ended at {min(crc_errors)}")