#!/usr/bin/env python3

import rospy
from apriltag_msgs.msg import ApriltagArrayStamped

tolerance = 500 # TODO: Get this from params

def tag_detection_cb(msg: ApriltagArrayStamped, camera):
    cameras_last_sent[camera] = rospy.Time.now()
    
cameras = ["ov2311_10_9_0_10_video0", "ov2311_10_9_0_10_video1", "ov2311_10_9_0_9_video0", "ov2311_10_9_0_9_video1"] # TODO: Get this from params 
cameras_last_sent = {}
camera_pubs = {}

if __name__ == '__main__':
    rospy.init_node('tags_failsafe_node')

    for camera in cameras:
        rospy.Subscriber(f"apriltag_detection_{camera}/tags", ApriltagArrayStamped, lambda msg: tag_detection_cb(msg, camera))
        cameras_last_sent[camera] = rospy.Time.now()
        camera_pubs[camera] = rospy.Publisher(f"apriltag_detection_{camera}/tags")

    while not rospy.is_shutdown():
        if rospy.Time.now() - cameras_last_sent[camera] > tolerance:
            rospy.logwarn_throttle(1.0, f'tags_failsafe_node: camera "{camera}" offline! publishing empty tag array')
            msg = ApriltagArrayStamped()
            msg.tags = []
            camera_pubs[camera].publish(msg)