#!/usr/bin/env python3
# node that subscribes to two float topics and then publishes a static transform
# transform should be from base_link to trap_scoring_frame and have a default value of 
#         <node pkg="tf2_ros" type="static_transform_publisher" name="baselink_to_trap_scoring" args="0.35 -0.08 0 0 0 0 1 base_link trap_scoring_frame" />
# we are in ros1

import rospy
import tf2_ros
import tf2_geometry_msgs
import geometry_msgs.msg
import std_msgs.msg
import math

class DynamicTrapTF:
    def __init__(self):
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.br = tf2_ros.StaticTransformBroadcaster()
        self.sub = rospy.Subscriber('/trap_x', std_msgs.msg.Float64, self.callback)
        self.trap_pose = geometry_msgs.msg.PoseStamped()
        self.trap_pose.pose.position.x = 0.38 # 0.36 worked on practice field Saturday, but the tag on the field is a bit further back
        self.trap_pose.pose.position.y = -0.08
        self.trap_pose.pose.position.z = 0.0
        self.trap_pose.pose.orientation.x = 0.0
        self.trap_pose.pose.orientation.y = 0.0
        self.trap_pose.pose.orientation.z = 0.0
        self.trap_pose.pose.orientation.w = 1.0
        self.tf_changed = True

    def callback(self, data):
        rospy.loginfo("Received trap_x: %f", data.data)
        self.trap_pose.pose.position.x = data.data
        self.tf_changed = True

    def run(self):
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            if self.tf_changed:
                self.tf_changed = False
                # publish the transform
                t = geometry_msgs.msg.TransformStamped()
                t.header.stamp = rospy.Time.now()
                t.header.frame_id = "base_link"
                t.child_frame_id = "trap_scoring_frame"
                t.transform.translation = self.trap_pose.pose.position
                t.transform.rotation = self.trap_pose.pose.orientation
                self.br.sendTransform(t)
                rospy.loginfo("Published transform from base_link to trap_scoring_frame with translation: " + str(t.transform.translation) + " and rotation: " + str(t.transform.rotation))
                rate.sleep()


if __name__ == '__main__':
    rospy.init_node('dynamic_trap_tf')
    node = DynamicTrapTF()
    node.run()

            