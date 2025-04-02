#!/usr/bin/env python3

import rospy
from apriltag_msgs.msg import ApriltagArrayStamped

tolerance = rospy.get_param("tolerance", 0.5) # seconds
rate = rospy.get_param("rate", 100) # Hz

def tag_detection_cb(msg: ApriltagArrayStamped, camera):
    if camera.frameid != f"fake_{camera}":
        cameras_last_sent[camera] = rospy.Time.now()
    
cameras = rospy.get_param("camera_names", ["ov2311_10_9_0_10_video0", "ov2311_10_9_0_10_video1", "ov2311_10_9_0_9_video0", "ov2311_10_9_0_9_video1"])
cameras_last_sent: dict[str, int] = {}
camera_pubs: dict[str, rospy.Publisher] = {}

if __name__ == '__main__':
    r = rospy.Rate(rate)
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
            msg.frameid = f"fake_{camera}"
            camera_pubs[camera].publish(msg)
        r.sleep()