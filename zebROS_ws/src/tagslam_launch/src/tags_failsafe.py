#!/usr/bin/env python3

import rospy
from apriltag_msgs.msg import ApriltagArrayStamped

tolerance = rospy.get_param("tolerance", 0.5) # seconds
rate = rospy.get_param("rate", 60) # Hz

def tag_detection_cb(camera: str):
    def internal_tag_detection_cb(msg: ApriltagArrayStamped):
        if msg.header.frame_id != f"fake_{camera}":
            cameras_last_sent[camera] = rospy.Time.now()
    return internal_tag_detection_cb

cameras = rospy.get_param("camera_names", ["ov2311_10_9_0_10_video0", "ov2311_10_9_0_10_video1", "ov2311_10_9_0_9_video0", "ov2311_10_9_0_9_video1"])
cameras_last_sent: dict[str, int] = {}
camera_pubs: dict[str, rospy.Publisher] = {}
camera_subs: dict[str, rospy.Subscriber] = {}
last_logged: dict[str, rospy.Time] = {}

if __name__ == '__main__':
    rospy.init_node('tags_failsafe_node')

    r = rospy.Rate(rate)

    for camera in cameras:
        print(f"Setting up {camera}")
        # so this works but lambda msg: tag_detection_cb(msg: ApriltagArrayStamped, camera: str) does weird things?!?
        # maybe camera is captured from here so you get the last one idk
        camera_subs[camera] = rospy.Subscriber(f"/apriltag_detection_{camera}/tags", ApriltagArrayStamped, tag_detection_cb(camera), queue_size=1, tcp_nodelay=True)
        cameras_last_sent[camera] = rospy.Time.now()
        camera_pubs[camera] = rospy.Publisher(f"/apriltag_detection_{camera}/tags", ApriltagArrayStamped, queue_size=1, tcp_nodelay=True)
        last_logged[camera] = rospy.Time.now()

    while not rospy.is_shutdown():
        for camera in cameras:
            if (rospy.Time.now() - cameras_last_sent[camera]).to_sec() > tolerance:
                empty_tags = ApriltagArrayStamped()
                empty_tags.apriltags = []
                empty_tags.header.frame_id = f"fake_{camera}"
                empty_tags.header.stamp = rospy.Time.now()
                if (rospy.Time.now() - last_logged[camera]).to_sec() > 1.0:
                    rospy.logwarn(f"tags_failsafe_node: camera {camera} offline! publishing empty tags to {camera_pubs[camera].resolved_name}")
                    last_logged[camera] = rospy.Time.now()
                camera_pubs[camera].publish(empty_tags)
        r.sleep()