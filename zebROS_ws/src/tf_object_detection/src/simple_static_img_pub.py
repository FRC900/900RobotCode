#!/usr/bin/env python3

import rospy
import sys
import rospkg
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

test_image_path = "/home/ubuntu/tensorflow_workspace/2023Game/data/videos/162_36_Angle.png"
img = cv2.imread(test_image_path)

rospy.init_node('pub_img', anonymous=False)
bridge = CvBridge()

image_pub = rospy.Publisher("/zed_objdetect/left/image_rect_color", Image, queue_size=1)

r = rospy.Rate(30)
while not rospy.is_shutdown():
    image_pub.publish(bridge.cv2_to_imgmsg(img, encoding="bgr8"))
    r.sleep()