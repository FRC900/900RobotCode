#!/usr/bin/env python3

import rospy
import sys
import cv2
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from camera_info_manager import CameraInfoManager

rospy.init_node('pub_img', anonymous=False)
image_pub = rospy.Publisher("image_raw", Image, queue_size=1)
camera_info_pub = rospy.Publisher("camera_info", CameraInfo, queue_size=1)

test_image_path = "/home/ubuntu/tensorflow_workspace/2023Game/data/videos/162_36_Angle.png"
test_image_path = rospy.get_param("~test_image_path", test_image_path)
rospy.loginfo(f"Publishing image from: {test_image_path}")
img = cv2.imread(test_image_path)
img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
if len(img.shape) == 2:
    encoding="mono8"
elif img.shape[2] == 3:
    encoding="bgr8"
elif img.shape[2] == 4:
    encoding="bgra8"
else:
    print("Invalid number of channels in image")
    sys.exit(1)

camera_name = "ov2311"
camera_name = rospy.get_param("~camera_name", camera_name)  
camera_info_manager = CameraInfoManager(camera_name)
camera_info_manager.loadCameraInfo()
camera_info_msg = camera_info_manager.getCameraInfo()
camera_info_msg.header.frame_id = camera_name

bridge = CvBridge()
imgmsg = bridge.cv2_to_imgmsg(img, encoding=encoding)
imgmsg.header.frame_id = camera_name

r = rospy.Rate(30)
while not rospy.is_shutdown():
    t = rospy.Time.now()
    imgmsg.header.stamp = t
    image_pub.publish(imgmsg)
    camera_info_msg.header.stamp = t
    camera_info_pub.publish(camera_info_msg)

    r.sleep()