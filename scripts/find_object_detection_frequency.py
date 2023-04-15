#!/usr/bin/env python3
import rospy
import rosbag
from rospy_message_converter import message_converter
import sys

bagfiles = sys.argv[1:]
objdet = '/tf_object_detection/object_detection_world'

det_dict = {}

i = 0

for bagfile_name in bagfiles:
    print(f"Opening {bagfile_name}, {i}/{len(bagfiles)} bags processed")
    bag = rosbag.Bag(bagfile_name)

    for topic, msg, t in bag.read_messages([objdet]):
        m = message_converter.convert_ros_message_to_dictionary(msg)
        this_time = t.to_sec()

        for obj in m['objects']:
            if obj['id'] in det_dict.keys():
                det_dict[obj['id']] += 1
            else:
                det_dict[obj['id']] = 1
    i += 1

print(det_dict)
print(list(map(lambda k: (k, det_dict[k]), sorted(det_dict, key=lambda k: det_dict[k], reverse=True))))