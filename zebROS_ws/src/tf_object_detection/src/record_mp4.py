#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2, sys, yaml
import numpy as np

rospy.init_node("pickles", anonymous=True)

topic = sys.argv[1]

out_mp4 = sys.argv[2]

bridge = CvBridge()

count = 0

fourcc = cv2.VideoWriter_fourcc(*'mp4v')
out = cv2.VideoWriter(out_mp4, fourcc, 60, (1920, 1080))

def cb(msg):
    cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    out.write(cv_img)
    cv2.imshow("image", cv_img)
    cv2.waitKey(1)

sub = rospy.Subscriber("/tf_object_detection/debug_image", Image, callback=cb)

rospy.spin()

out.release()