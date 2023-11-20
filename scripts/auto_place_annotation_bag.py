import rospy, rosbag
from sensor_msgs.msg import Image
from frc_msgs.msg import JoystickState
from cv_bridge import CvBridge
import cv2, sys, yaml
import numpy as np
import math
from sensor_msgs.msg import CameraInfo
import image_geometry

camera_info = CameraInfo()

camera_info.header.frame_id = "zed_objdetect_left_camera_optical_frame"
camera_info.height = 720
camera_info.width = 1280
camera_info.distortion_model = "plumb_bob"
camera_info.D = [0.0, 0.0, 0.0, 0.0, 0.0]
camera_info.K = [523.6636962890625, 0.0, 626.8062744140625, 0.0, 523.6636962890625, 350.36322021484375, 0.0, 0.0, 1.0]
camera_info.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
camera_info.P = [523.6636962890625, 0.0, 626.8062744140625, 0.0, 0.0, 523.6636962890625, 350.36322021484375, 0.0, 0.0, 0.0, 1.0, 0.0]
camera_info.binning_x = 0
camera_info.binning_y = 0

pcm = image_geometry.PinholeCameraModel()
pcm.fromCameraInfo(camera_info)

# here's an idea: record /tag_detections instead of /tf_object_detection/tag_detection_world next time... :)

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

has_cmd_vel = False
has_teleop_vel = False
teleop_vel_topic = "/teleop/swerve_drive_controller/cmd_vel"
tag_topic = "/tag_detections"
has_good_tags = False

info_dict = yaml.load(bag._get_yaml_info())
for t in info_dict['topics']:
    if t['topic'] == image_topic:
        fps = t['frequency']
    if t['topic'] == "/frcrobot_jetson/swerve_drive_controller/cmd_vel_out":
        has_cmd_vel = True
    if t['topic'] == teleop_vel_topic:
        has_teleop_vel = True
    if t['topic'] == tag_topic:
        has_good_tags = True

cmd_vel_topic = "/frcrobot_jetson/swerve_drive_controller/cmd_vel_out"
bbox = "/frcrobot_rio/button_box_states"
joystick = "/frcrobot_rio/joystick_states1"
apriltag = "/tf_object_detection/tag_detection_world"

_, msg, _ = next(bag.read_messages(topics=[image_topic]))
cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="bgra8")

fourcc = cv2.VideoWriter_fourcc(*'mp4v')
out = cv2.VideoWriter(out_mp4, fourcc, fps, (cv_img.shape[1], cv_img.shape[0]))

joy = False
auto = False
moving = True

joystick_img = cv2.imread("/home/ubuntu/2023RobotCode/scripts/button_imgs/joystick.png", cv2.IMREAD_UNCHANGED)
robot_img = cv2.imread("/home/ubuntu/2023RobotCode/scripts/button_imgs/robot_1f916.png", cv2.IMREAD_UNCHANGED)

apriltag_locations = {"6": (0, 0)}
auto_samples = 0

for topic, msg, t in bag.read_messages(topics=[image_topic, joystick, apriltag, cmd_vel_topic, teleop_vel_topic, tag_topic]):
    if topic == joystick:
        joy = (math.hypot(msg.leftStickX, msg.leftStickY) + abs(msg.rightStickX)) > 0.01
        if not has_cmd_vel:
            auto = not joy
        continue
    if topic == apriltag:
        if not has_good_tags:
            apriltag_locations["6"] = (0, 0)
            for obj in msg.objects:
                apriltag_locations[obj.id] = tuple(map(int, pcm.project3dToPixel((-obj.location.y, -obj.location.z, obj.location.x))))[::-1] # see comment above
                print(apriltag_locations[obj.id])
        continue
    if topic == cmd_vel_topic:
        if ((math.hypot(msg.twist.linear.x, msg.twist.linear.y) + abs(msg.twist.angular.z)) > 0.01) and not joy:
            auto_samples += 1
        else:
            auto_samples = 0
        auto = auto_samples > 2
        # moving = ((math.hypot(msg.twist.linear.x, msg.twist.linear.y) + abs(msg.twist.angular.z)) > 0.01)
        continue
    if topic == teleop_vel_topic:
        # if ((math.hypot(msg.linear.x, msg.linear.y) + abs(msg.angular.z)) < 0.01) and moving and not joy:
        #     auto_samples += 1
        # else:
        #     auto_samples = 0
        # auto = auto_samples > 2
        # if auto:
        #     print(f"{msg.linear.x}, {msg.linear.y}, {msg.angular.z}, {moving}, {joy}")
        continue
    if topic == tag_topic:
        apriltag_locations["6"] = (0, 0)
        for det in msg.detections:
            avgX = 0
            avgY = 0
            for corn in det.corners:
                avgX += corn.x
                avgY += corn.y
            avgX /= 4
            avgY /= 4
            apriltag_locations[str(det.id[0])] = (int(avgX), int(avgY))
            print(f"{det.id[0]} at {apriltag_locations[str(det.id[0])]}")
        continue

    cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    if joy:
        x_offset = 50
        y_offset = 50
        cv_img = overlay_transparent(cv_img, joystick_img, x_offset, y_offset)
    elif auto:
        x_offset = 50
        y_offset = 50
        cv_img = overlay_transparent(cv_img, robot_img, x_offset, y_offset)

    # tag location is off for some reason
    if has_good_tags:
        if apriltag_locations["6"] != (0, 0):
            cv2.circle(cv_img, apriltag_locations["6"], 10, (0xFF, 0x70, 0x88), -1)

    out.write(cv_img[:, :, :3])
    cv2.imshow("image", cv_img)
    cv2.waitKey(1)

    count += 1

    if count % 10 == 0:
        print(f"{count} msgs done")

out.release()