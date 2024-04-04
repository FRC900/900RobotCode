#!/usr/bin/env python3
"""
Node that subscribes to /bagged_tf and republishes transforms from specific child frames
to the /tf topic.
"""

import rospy
import tf2_ros
import tf2_msgs.msg
import geometry_msgs.msg
import tf2_geometry_msgs

class FilterTransforms:
    def __init__(self):
        rospy.init_node('filter_transforms')
        self.bag_tf_sub = rospy.Subscriber('/bagged_tf', tf2_msgs.msg.TFMessage, self.tf_callback)
        self.tf_pub = rospy.Publisher('/tf', tf2_msgs.msg.TFMessage, queue_size=10)

        self.valid_frame_combos = [["odom", "base_link"]]

    def tf_callback(self, msg: tf2_msgs.msg.TFMessage):
        for transform in msg.transforms:
            # print(transform)
            if [transform.header.frame_id, transform.child_frame_id] in self.valid_frame_combos:
                self.tf_pub.publish(msg)


if __name__ == '__main__':
    ft = FilterTransforms()
    rospy.spin()