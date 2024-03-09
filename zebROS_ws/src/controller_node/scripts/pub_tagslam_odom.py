#!/usr/bin/env python3
# Script to subscribe to the transform from map to body_link, and publish it as a nav_msgs/Odometry message
# This is used to visualize the robot's position in RViz

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Point, Quaternion, Twist, TwistWithCovariance, Vector3
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class PubTagslamOdom:
    def __init__(self):
        rospy.init_node('pub_tagslam_odom')
        self.odom_pub = rospy.Publisher('/tagslam/odom/body_frc_robot', Odometry, queue_size=1)
        self.listener = tf.TransformListener()
        self.rate = rospy.Rate(250.0)
        self.odom = Odometry()
        self.odom.header.frame_id = "map"
        self.odom.child_frame_id = "base_link"

    def run(self):
        while not rospy.is_shutdown():
            try:
                trans, rot = self.listener.lookupTransform('map', 'body_link', rospy.Time(0))
                self.odom.header.stamp = rospy.Time.now()
                self.odom.pose.pose.position.x = trans[0]
                self.odom.pose.pose.position.y = trans[1]
            except:
                pass
            self.odom_pub.publish(self.odom)
            self.rate.sleep()

if __name__ == '__main__':
    pub_tagslam_odom = PubTagslamOdom()
    pub_tagslam_odom.run()