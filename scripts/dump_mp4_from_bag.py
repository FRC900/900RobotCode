import rospy, rosbag
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2, sys, yaml
import numpy as np

bagfile_name = sys.argv[1]

topic = sys.argv[2]

out_mp4 = sys.argv[3]

bag = rosbag.Bag(bagfile_name, "r")
bridge = CvBridge()

count = 0

info_dict = yaml.load(bag._get_yaml_info())
for t in info_dict['topics']:
    if t['topic'] == topic:
        fps = t['frequency']

_, msg, _ = next(bag.read_messages(topics=[topic]))
cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

fourcc = cv2.VideoWriter_fourcc(*'mp4v')
out = cv2.VideoWriter(out_mp4, fourcc, fps, (cv_img.shape[1], cv_img.shape[0]))

for topic, msg, t in bag.read_messages(topics=[topic]):
    cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    out.write(cv_img)
    cv2.imshow("image", cv_img)
    cv2.waitKey(1)

    count += 1

    if count % 10 == 0:
        print(f"{count} msgs done")

out.release()