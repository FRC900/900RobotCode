#!/usr/bin/env python3

import rospy
import tf2_ros
from tf.transformations import euler_from_quaternion
from candle_controller_msgs.srv import Colour, ColourRequest
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Imu
from frc_msgs.msg import MatchSpecificData
from time import sleep
from candle_controller_msgs.srv import ColourArray, ColourArrayRequest
from candle_controller_msgs.msg import CANdleColour
from apriltag_msgs.msg import ApriltagArrayStamped

ORANGE = (255, 165, 0)
status_array = [ORANGE] * 43

DOT9V0 = 0
DOT9V1 = 1
DOT10V0 = 2
DOT10V1 = 3
COLOR_MATCH_DATA = 4
BLINK_STATE = True

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

def set_blink(msg):
    global BLINK_STATE
    rospy.loginfo("inverted blink state")
    BLINK_STATE = not BLINK_STATE

def blink_idx(idx, color):
    global BLINK_STATE
    if BLINK_STATE:
        status_array[idx] = color
    else:
        status_array[idx] = (0, 0, 0) 

def match_data_callback(data):
    print("match data callback")
    if data.allianceColor == MatchSpecificData.ALLIANCE_COLOR_RED:
        status_array[COLOR_MATCH_DATA] = (255,0,0)
        print("red")
    elif data.allianceColor == MatchSpecificData.ALLIANCE_COLOR_BLUE:
        status_array[COLOR_MATCH_DATA] = (0,0,255)
        print("blue")
    elif data.allianceColor == MatchSpecificData.ALLIANCE_COLOR_UNKNOWN:
        status_array[COLOR_MATCH_DATA] = (255,255,255)
        print("unk")
    else:
        rospy.loginfo("match data bad")

def make_colour_obj(start, count, r, g, b):
    colour = ColourRequest()
    colour.start = start
    colour.count = count
    colour.red = r
    colour.green = g
    colour.blue = b
    colour.white = 0
    return colour

def camera_cb(msg, args):
    idx = args 
    status_array[idx] = (0, 255, 0)

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
    rospy.wait_for_service('/frcrobot_jetson/candle_controller/colour_array')
    colour_client = rospy.ServiceProxy('/frcrobot_jetson/candle_controller/colour_array', ColourArray)
    blink_timer = rospy.Timer(period=rospy.Duration(1.0/2.0), callback=set_blink)


    dot9v0sub = rospy.Subscriber("/apriltag_detection_ov2311_10_9_0_9_video0/tags", ApriltagArrayStamped, camera_cb, (DOT9V0))
    dot9v1sub = rospy.Subscriber("/apriltag_detection_ov2311_10_9_0_9_video1/tags", ApriltagArrayStamped, camera_cb, (DOT9V1))
    dot10v0sub = rospy.Subscriber("/apriltag_detection_ov2311_10_9_0_10_video0/tags", ApriltagArrayStamped, camera_cb, (DOT10V0))
    dot10v1sub = rospy.Subscriber("/apriltag_detection_ov2311_10_9_0_10_video1/tags", ApriltagArrayStamped, camera_cb, (DOT10V1))
    r = rospy.Rate(10)
    led_arr_msg = ColourArrayRequest()
    for idx, i in enumerate(status_array):
        
        colour_srv = CANdleColour()
        colour_srv.start = 8 + idx
        colour_srv.count = 1
        colour_srv.red = 255
        colour_srv.green = 255
        colour_srv.blue = 255
        led_arr_msg.colours.append(colour_srv)   
    colour_client(led_arr_msg)

    while not rospy.is_shutdown():
        led_arr_msg = ColourArrayRequest()
        for idx, i in enumerate(status_array):
            
            colour_srv = CANdleColour()
            colour_srv.start = 8 + idx
            colour_srv.count = 1
            colour_srv.red = i[0]
            colour_srv.green = i[1]
            colour_srv.blue = i[2]
            if idx == len(status_array) - 1:
                #print("Here!")
                colour_srv.red = 255
                colour_srv.green = 255
                colour_srv.blue = 255

            led_arr_msg.colours.append(colour_srv)

        #print(led_arr_msg)
        colour_client(led_arr_msg)
        for idx in [DOT9V0, DOT9V1, DOT10V0, DOT10V1, COLOR_MATCH_DATA]:
            blink_idx(idx, ORANGE)
        if is_enabled:
            break
        r.sleep()