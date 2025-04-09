#!/usr/bin/env python3
import rosbag
import tf
from tf.msg import tfMessage
import rospy
from geometry_msgs.msg import TransformStamped
import sys

def combine_bags(input_bag1, input_bag2, output_bag):
    # Open the input bags
    bag1 = rosbag.Bag(input_bag1, 'r')
    bag2 = rosbag.Bag(input_bag2, 'r')

    # Open the output bag
    out_bag = rosbag.Bag(output_bag, 'w')

    # Read messages from bag1
    for topic, msg, t in bag1.read_messages(topics=[]):
        # Write to output bag
        out_bag.write(topic, msg, t)
        
    # Read messages from bag2
    for topic, msg, t in bag2.read_messages(topics=['/tf']):
        new_tfs = []
        for transform in msg.transforms:
            # Check if either the source or the target frame matches the target frame
            transform.child_frame_id += "_"
            if transform.header.frame_id != "map":
                transform.header.frame_id += "_"
        # Write to output bag
        out_bag.write(topic, msg, t)

    # Close bags
    bag1.close()
    bag2.close()
    out_bag.close()

if __name__ == '__main__':
    input_bag1 = sys.argv[1]
    input_bag2 = sys.argv[2]
    output_bag = sys.argv[3]

    rospy.init_node('combine_rosbags')

    combine_bags(input_bag1, input_bag2, output_bag)
    print(f"Transforms from {input_bag1} and {input_bag2} have been combined into {output_bag}")