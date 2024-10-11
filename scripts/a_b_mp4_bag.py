import rospy, rosbag
from sensor_msgs.msg import Image
from frc_msgs.msg import JoystickState
from cv_bridge import CvBridge
import cv2, sys, yaml
import numpy as np
import math

def overlay_transparent(background, overlay, x, y):

    background_width = background.shape[1]
    background_height = background.shape[0]

    if x >= background_width or y >= background_height:
        return background

    h, w = overlay.shape[0], overlay.shape[1]

    if x + w > background_width:
        w = background_width - x
        overlay = overlay[:, :w]

    if y + h > background_height:
        h = background_height - y
        overlay = overlay[:h]

    if overlay.shape[2] < 4:
        overlay = np.concatenate(
            [
                overlay,
                np.ones((overlay.shape[0], overlay.shape[1], 1), dtype = overlay.dtype) * 255
            ],
            axis = 2,
        )

    overlay_image = overlay[..., :3]
    mask = overlay[..., 3:] / 255.0

    background[y:y+h, x:x+w] = (1.0 - mask) * background[y:y+h, x:x+w] + mask * overlay_image

    return background

bagfile_name = sys.argv[1]

image_topic = sys.argv[2]

out_mp4 = sys.argv[3]

bag = rosbag.Bag(bagfile_name, "r")
bridge = CvBridge()

count = 0

info_dict = yaml.load(bag._get_yaml_info())
for t in info_dict['topics']:
    if t['topic'] == image_topic:
        fps = t['frequency']

joystick = "/frcrobot_rio/joystick_states1"

_, msg, _ = next(bag.read_messages(topics=[image_topic]))
cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="bgra8")

fourcc = cv2.VideoWriter_fourcc(*'mp4v')
out = cv2.VideoWriter(out_mp4, fourcc, fps, (cv_img.shape[1], cv_img.shape[0]))

a = False
b = False

a_img = cv2.imread("/home/ubuntu/900RobotCode/scripts/button_imgs/a_button.png", cv2.IMREAD_UNCHANGED)
b_img = cv2.imread("/home/ubuntu/900RobotCode/scripts/button_imgs/b_button.png", cv2.IMREAD_UNCHANGED)

print(a_img.shape)

for topic, msg, t in bag.read_messages(topics=[image_topic, joystick]):
    if topic == joystick:
        a = msg.buttonAButton and msg.rightStickX < 0.01
        b = msg.buttonBButton and msg.rightStickX < 0.01
        continue

    cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    if a:
        x_offset = 50
        y_offset = 50
        cv_img = overlay_transparent(cv_img, a_img, x_offset, y_offset)
    if b:
        x_offset = 100
        y_offset = 50
        cv_img = overlay_transparent(cv_img, b_img, x_offset, y_offset)

    out.write(cv_img[:, :, :3])
    cv2.imshow("image", cv_img)
    cv2.waitKey(1)

    count += 1

    if count % 10 == 0:
        print(f"{count} msgs done")

out.release()