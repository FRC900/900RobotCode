import os
import rospy
import rosbag
from rospy_message_converter import message_converter
import sys

#Bagfile name from command line
bagfile_name = sys.argv[1]

bag = rosbag.Bag(bagfile_name)

tf_static_topic = '/tf_static'

printed_transforms = set()
for topic, msg, t in bag.read_messages([tf_static_topic]):
    m = message_converter.convert_ros_message_to_dictionary(msg)
    #print(m)

    for t in m['transforms']:
        from_name = t['header']['frame_id']
        to_name = t['child_frame_id']
        if from_name+to_name not in printed_transforms:
            printed_transforms.add(from_name+to_name)

            trans_x = t['transform']['translation']['x']
            trans_y = t['transform']['translation']['y']
            trans_z = t['transform']['translation']['z']
            rot_w = t['transform']['rotation']['w']
            rot_x = t['transform']['rotation']['x']
            rot_y = t['transform']['rotation']['y']
            rot_z = t['transform']['rotation']['z']

            print(f'<node pkg="tf" type="static_transform_publisher" name="{from_name}_to_{to_name}" args="{trans_x} {trans_y} {trans_z} {rot_w} {rot_x} {rot_y} {rot_z} {from_name} {to_name} 100')


