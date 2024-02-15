#!/usr/bin/env python3

import rospy
import tf2_ros
from tf.transformations import euler_from_quaternion
from candle_controller_msgs.srv import Colour, ColourRequest
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Imu
from frc_msgs.msg import MatchSpecificData
from time import sleep

def imu_callback(imu):
    global orientation
    orientation = imu.orientation

def wanted_point_callback(pose):
    global wanted_x
    global wanted_y
    global wanted_r
    wanted_x = pose.position.x
    wanted_y = pose.position.y
    wanted_r = euler_from_quaternion([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])

def match_data_callback(data):
    global is_enabled
    is_enabled = data.Enabled

def make_colour_obj(start, count, r, g, b):
    colour = ColourRequest()
    colour.start = start
    colour.count = count
    colour.red = r
    colour.green = g
    colour.blue = b
    colour.white = 0
    return colour

def send_colours(x_col, y_col, r_col):
    rospy.wait_for_service('/frcrobot_jetson/candle_controller/colour')
    try:
        colour_client = rospy.ServiceProxy('/frcrobot_jetson/candle_controller/colour', Colour)
        # Start/counts should be edited to match the real robot
        x_colour = make_colour_obj(9, 2, x_col[0], x_col[1], x_col[2])
        colour_client(x_colour)
        sleep(0.1) # Cameron's fault
        y_colour = make_colour_obj(11, 2, y_col[0], y_col[1], y_col[2])
        colour_client(y_colour)
        sleep(0.1)
        r_colour = make_colour_obj(13, 2, r_col[0], r_col[1], r_col[2])
        colour_client(r_colour)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

RED = [255, 0, 0]
GREEN = [0, 255, 0]
BLUE = [0, 0, 255]
wanted_x = None
orientation = None
is_enabled = False
if __name__ == "__main__":
    rospy.init_node("pregame_candle")
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    orientation_sub = rospy.Subscriber("/imu/zeroed_imu", Imu, imu_callback)
    wanted_point_sub = rospy.Subscriber("/auto/first_point", Pose, wanted_point_callback)
    match_data_sub = rospy.Subscriber("/frcrobot_rio/match_data", MatchSpecificData, match_data_callback)
    r = rospy.Rate(10)

    while not rospy.is_shutdown():
        if wanted_x is not None:
            trans = tfBuffer.lookup_transform('map', 'base_link', rospy.Time())
            print(trans.transform.translation.x, wanted_x)
            if abs(trans.transform.translation.x - wanted_x) < 0.05:
                x_col = GREEN
            else:
                x_col = RED
            if abs(trans.transform.translation.y - wanted_y) < 0.05:
                y_col = GREEN
            else:
                y_col = RED
            if orientation is not None:
                euler_orientation = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
                if abs(euler_orientation[2] - wanted_r[2]) < 0.05:
                    r_col = GREEN
                else:
                    r_col = RED
            send_colours(x_col, y_col, r_col)
            
        if is_enabled:
            break
        r.sleep()