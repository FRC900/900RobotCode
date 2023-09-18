#! /usr/bin/env python3
# Copyright (c) 2015, Rethink Robotics, Inc.

# Using this CvBridge Tutorial for converting
# ROS images to OpenCV2 images
# http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython

# Using this OpenCV2 tutorial for saving Images:
# http://opencv-python-tutroals.readthedocs.org/en/latest/py_tutorials/py_gui/py_image_display/py_image_display.html

# rospy for the subscriber
import rospy
# ROS Image message
from sensor_msgs.msg import Image
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2

# Various contants
# TODO : make them config items or params or something
SECONDS_PER_WRITE = 2.0
FILE_PREFIX = '/home/ubuntu/tags/tag8_'

# Instantiate CvBridge
bridge = CvBridge()
last_write_time = None

def image_callback(msg):
    global bridge
    global last_write_time
    try:
        # Convert your ROS Image message to OpenCV2
        cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
    except (CvBridgeError, e):
        print(e)
    else:
        if (rospy.get_time() - last_write_time) > SECONDS_PER_WRITE:
            time = msg.header.stamp
            cv2.imwrite(FILE_PREFIX+str(int(time.to_sec()))+'.png', cv2_img)
            last_write_time = rospy.get_time()

def main():
    rospy.init_node('image_listener')
    # Define your image topic
    image_topic = "/zed_objdetect/left/image_rect_color"
    global last_write_time
    last_write_time = rospy.get_time()

    # Set up your subscriber and define its callback
    rospy.Subscriber(image_topic, Image, image_callback)
    # Spin until ctrl + c
    rospy.spin()

if __name__ == '__main__':
    main()