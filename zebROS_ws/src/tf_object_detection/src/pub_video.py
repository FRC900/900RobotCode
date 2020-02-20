#!/usr/bin/env python

import rospy
import sys
import rospkg
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

show_video = False

class Video2ROS:
    def __init__(self):
        rospy.init_node('pub_video', anonymous=False)
        rospy.on_shutdown(self.cleanup)

        self.filename = "video.mp4"
        self.pub_topic = "c920/rect_image"
        self.framerate = 30
        self.show_video = False

        if rospy.has_param('pub_topic'):
            self.pub_topic = rospy.get_param('pub_topic')

        if rospy.has_param('filename'):
            self.filename = rospy.get_param('filename')

        if rospy.has_param('framerate'):
            self.framerate = rospy.get_param('framerate')

        if rospy.has_param('show_video'):
            self.show_video = rospy.get_param('show_video')

        rospack = rospkg.RosPack()
        image_path = rospack.get_path('tf_object_detection') + '/src/'
        self.capture = cv2.VideoCapture(image_path + str(self.filename))
        bridge = CvBridge()

        image_pub = rospy.Publisher(self.pub_topic, Image, queue_size=10)

        while not rospy.is_shutdown():
            ret, frame = self.capture.read()
            try:
                image_pub.publish(bridge.cv2_to_imgmsg(frame, encoding="bgr8"))
            except CvBridgeError, e:
                print e

            if show_video:
                display_image = frame.copy()
                cv2.imshow("Video Playback", display_image)

            self.keystroke = cv2.waitKey(1000 / self.framerate)

    def cleanup(self):
            print "Shutting down video pub node."
            cv2.destroyAllWindows()

def main(args):
    try:
        v2r = Video2ROS()
    except KeyboardInterrupt:
        v2r.cleanup()

if __name__ == '__main__':
    main(sys.argv)
