#!/usr/bin/python3
import cv2
import sys
import rospy
from sensor_msgs.msg import Image
from sensor_msgs.srv import SetCameraInfo
from sensor_msgs.msg import CameraInfo

"""
use what is printed out when clicking "commit" in the calibration utility

camera_info: 
  header: 
    seq: 0
    stamp: 
      secs: 0
      nsecs:         0
    frame_id: ''
  height: 480
  width: 640
  distortion_model: "plumb_bob"
  D: [0.04507368872502638, -0.19661785523443845, 0.001512452282124494, 0.004642557892851897, 0.0]
  K: [772.0256907026344, 0.0, 342.98345181074393, 0.0, 776.8330401937692, 301.4710831302156, 0.0, 0.0, 1.0]
  R: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
  P: [771.5826416015625, 0.0, 344.5282774027328, 0.0, 0.0, 777.8756713867188, 301.43318857727536, 0.0, 0.0, 0.0, 1.0, 0.0]
  binning_x: 0
  binning_y: 0
  roi: 
    x_offset: 0
    y_offset: 0
    height: 0
    width: 0
    do_rectify: False
"""

camera_info = CameraInfo()
camera_info.height = 480
camera_info.width = 640
camera_info.distortion_model = "plumb_bob"
camera_info.D = [0.04507368872502638, -0.19661785523443845, 0.001512452282124494, 0.004642557892851897, 0.0]
camera_info.K = [772.0256907026344, 0.0, 342.98345181074393, 0.0, 776.8330401937692, 301.4710831302156, 0.0, 0.0, 1.0]
camera_info.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
camera_info.P = [771.5826416015625, 0.0, 344.5282774027328, 0.0, 0.0, 777.8756713867188, 301.43318857727536, 0.0, 0.0, 0.0, 1.0, 0.0]

if len(sys.argv) < 2:
    print("Usage: publish_from_camera.py [camera ID]")
    sys.exit(1)

cap = cv2.VideoCapture(int(sys.argv[1]), cv2.CAP_V4L2)
from cv_bridge import CvBridge
bridge = CvBridge()

rospy.init_node("publish_from_camera", anonymous=True)
pub = rospy.Publisher("/zed_objdetect/left/image_rect_color", Image, queue_size=1)
pub_info = rospy.Publisher("/zed_objdetect/left/camera_info", CameraInfo, queue_size=1)
s = rospy.Service('/zed_objdetect/left/set_camera_info', SetCameraInfo, lambda req: print(req))

r = rospy.Rate(100)

while not rospy.is_shutdown():
    print("img")
    ret, img = cap.read()
    print("img read")
    if not ret: break
    img_msg = bridge.cv2_to_imgmsg(img, encoding="bgr8")
    img_msg.header.frame_id = "base_link" # without a zed running we don't have static transforms for it
    img_msg.header.stamp = rospy.Time.now()
    pub.publish(img_msg)
    camera_info.header = img_msg.header
    pub_info.publish(camera_info)
    print("published")
    r.sleep()
    print("slept")

cap.release()