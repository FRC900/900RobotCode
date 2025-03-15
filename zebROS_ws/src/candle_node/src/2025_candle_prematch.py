#!/usr/bin/env python3

import rospy
import tf2_ros
from tf.transformations import euler_from_quaternion
from candle_controller_msgs.srv import Colour, ColourRequest
from geometry_msgs.msg import PoseStamped, TransformStamped, Quaternion
from sensor_msgs.msg import Imu
from frc_msgs.msg import MatchSpecificData
from time import sleep
from candle_controller_msgs.srv import ColourArray, ColourArrayRequest
from candle_controller_msgs.msg import CANdleColour
from apriltag_msgs.msg import ApriltagArrayStamped
from sensor_msgs.msg import JointState
from angles import shortest_angular_distance
from math import hypot
from geometry_msgs.msg import TwistStamped
from behavior_actions.msg import AutoMode

ORANGE = (255, 165, 0)
GREEN = (0, 255, 0)
status_array = [ORANGE] * 43

DOT9V0 = 0
DOT9V1 = 1
DOT10V0 = 2
DOT10V1 = 3

COLOR_RAW_MATCH_DATA = 4

# have 2 LEDs on top of each other for used match data to differentiate from FMS one
COLOR_MATCH_DATA = 5
COLOR_MATCH_DATA_2 = 36

TAGSLAM_ALIVE = 6
IMU_CORRECT = 7

AUTO_POSITION_CLOSE = 8
AUTO_ROTATION_CLOSE = 9

ELEVATOR_AVOID_TRIGGERED = 10
HAVE_CORAL = 11

CMD_VEL_ZERO = 12
CORRECT_AUTO = 13

HEARTBEAT = 20 # last in row
BLINK_STATE = True

# ------------------
TAGSLAM_TIMEOUT = 10.0 # seconds
IMU_ZERO_ANGLE_THRESHOLD = 0.051 # radians, approx. 3 degrees

MAP_TO_BASE_LINK_TIMEOUT = 0.5 # seconds
AUTO_ANGLE_THRESHOLD = 0.17 # radians, approx. 10 degrees
AUTO_POSITION_THRESHOLD = 0.5 # meters (radius/hypotenuse)

ELEVATOR_AVOID_SWITCH_NAME = "elevator_avoid_limit_switch"
ROLLER_SWITCH_NAME = "roller_limit_switch"

elevator_avoid_was_triggered = False

imu_orientation: Quaternion = Quaternion(0,0,0,1)

def twist_callback(msg: TwistStamped):
    if abs(hypot(msg.twist.linear.x, msg.twist.linear.y)) < 0.05 and abs(msg.twist.angular.z) < 0.05:
        status_array[CMD_VEL_ZERO] = GREEN

def auto_mode_callback(msg: AutoMode):
    if msg.auto_mode == 1 or msg.auto_mode == 2:
        status_array[CORRECT_AUTO] = GREEN

def imu_callback(imu: Imu):
    global imu_orientation
    imu_orientation = imu.orientation

def wanted_point_callback(pose: PoseStamped):
    global wanted_x
    global wanted_y
    global wanted_r
    wanted_x = pose.pose.position.x
    wanted_y = pose.pose.position.y
    wanted_r = euler_from_quaternion([pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w])[2]

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

def raw_match_data_callback(data):
    #print("match data callback")
    if data.allianceColor == MatchSpecificData.ALLIANCE_COLOR_RED:
        status_array[COLOR_RAW_MATCH_DATA] = (255,0,0)
        #print("red")
    elif data.allianceColor == MatchSpecificData.ALLIANCE_COLOR_BLUE:
        status_array[COLOR_RAW_MATCH_DATA] = (0,0,255)
        #print("blue")
    elif data.allianceColor == MatchSpecificData.ALLIANCE_COLOR_UNKNOWN:
        status_array[COLOR_RAW_MATCH_DATA] = (255,255,255)
        #print("unk")
    else:
        rospy.loginfo("FMS match data bad")

def match_data_callback(data):
    #print("match data callback")
    if data.allianceColor == MatchSpecificData.ALLIANCE_COLOR_RED:
        status_array[COLOR_MATCH_DATA] = (255,0,0)
        #print("red")
    elif data.allianceColor == MatchSpecificData.ALLIANCE_COLOR_BLUE:
        status_array[COLOR_MATCH_DATA] = (0,0,255)
        #print("blue")
    elif data.allianceColor == MatchSpecificData.ALLIANCE_COLOR_UNKNOWN:
        status_array[COLOR_MATCH_DATA] = (255,255,255)
        #print("unk")
    else:
        rospy.loginfo("Republished match data bad")
    status_array[COLOR_MATCH_DATA_2] = status_array[COLOR_MATCH_DATA]

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
    status_array[idx] = GREEN

def limit_switch_callback(data: JointState):
    global elevator_avoid_was_triggered
    if ELEVATOR_AVOID_SWITCH_NAME in data.name:
        elevator_avoid_was_triggered = elevator_avoid_was_triggered or data.position[data.name.index(ELEVATOR_AVOID_SWITCH_NAME)]
    else:
        rospy.logwarn_throttle(1.0, f'2025_candle_prematch: elevator avoid switch "{ELEVATOR_AVOID_SWITCH_NAME}" not found')
        pass

    if elevator_avoid_was_triggered:
        status_array[ELEVATOR_AVOID_TRIGGERED] = GREEN

    if ROLLER_SWITCH_NAME in data.name:
        if data.position[data.name.index(ROLLER_SWITCH_NAME)]:
            status_array[HAVE_CORAL] = GREEN
    else:
        rospy.logwarn_throttle(1.0, f'2025_candle_prematch: roller switch "{ROLLER_SWITCH_NAME}" not found')
        pass

wanted_x = None
imu_orientation = None
is_enabled = False
if __name__ == "__main__":
    rospy.init_node("pregame_candle")
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    orientation_sub = rospy.Subscriber("/imu/zeroed_imu", Imu, imu_callback, tcp_nodelay=True)
    wanted_point_sub = rospy.Subscriber("/auto/first_point", PoseStamped, wanted_point_callback, tcp_nodelay=True)
    match_data_sub = rospy.Subscriber("/frcrobot_rio/match_data", MatchSpecificData, match_data_callback, tcp_nodelay=True)
    raw_match_data_sub = rospy.Subscriber("/frcrobot_rio/match_data_raw", MatchSpecificData, raw_match_data_callback, tcp_nodelay=True)
    limit_switch_sub = rospy.Subscriber("/frcrobot_rio/joint_states", JointState, limit_switch_callback, tcp_nodelay=True)
    rospy.wait_for_service('/frcrobot_jetson/candle_controller/colour_array')
    colour_client = rospy.ServiceProxy('/frcrobot_jetson/candle_controller/colour_array', ColourArray)
    blink_timer = rospy.Timer(period=rospy.Duration(1.0/2.0), callback=set_blink)

    dot9v0sub = rospy.Subscriber("/apriltag_detection_ov2311_10_9_0_9_video0/tags", ApriltagArrayStamped, camera_cb, (DOT9V0), tcp_nodelay=True)
    dot9v1sub = rospy.Subscriber("/apriltag_detection_ov2311_10_9_0_9_video1/tags", ApriltagArrayStamped, camera_cb, (DOT9V1), tcp_nodelay=True)
    dot10v0sub = rospy.Subscriber("/apriltag_detection_ov2311_10_9_0_10_video0/tags", ApriltagArrayStamped, camera_cb, (DOT10V0), tcp_nodelay=True)
    dot10v1sub = rospy.Subscriber("/apriltag_detection_ov2311_10_9_0_10_video1/tags", ApriltagArrayStamped, camera_cb, (DOT10V1), tcp_nodelay=True)

    cmd_vel_sub = rospy.Subscriber("/frcrobot_jetson/swerve_drive_controller/cmd_vel_out", TwistStamped, twist_callback, tcp_nodelay=True)
    auto_mode_sub = rospy.Subscriber("/auto/auto_mode", AutoMode, auto_mode_callback, tcp_nodelay=True)
    r = rospy.Rate(10)
    led_arr_msg = ColourArrayRequest()
    for idx, i in enumerate(status_array):
        
        colour_srv = CANdleColour()
        colour_srv.start = 0
        colour_srv.count = 7
        colour_srv.red = 0
        colour_srv.green = 0
        colour_srv.blue = 0
        led_arr_msg.colours.append(colour_srv)   
    try:
        colour_client(led_arr_msg)
    except:
        rospy.logerr_throttle(1.0, "2025_candle_prematch: unable to call LED service?")

    led_arr_msg = ColourArrayRequest()
    for idx, i in enumerate(status_array):
        
        colour_srv = CANdleColour()
        colour_srv.start = 8 + idx
        colour_srv.count = 1
        colour_srv.red = 255
        colour_srv.green = 255
        colour_srv.blue = 255
        led_arr_msg.colours.append(colour_srv)   
    
    try:
        colour_client(led_arr_msg)
    except:
        rospy.logerr_throttle(1.0, "2025_candle_prematch: unable to call LED service?")

    while not rospy.is_shutdown():
        # tf checks (non-callback)
        try:
            tagslam_tf: TransformStamped = tfBuffer.lookup_transform("map", "frc_robot", rospy.Time())
            if (rospy.Time().to_sec() - tagslam_tf.header.stamp.to_sec()) < TAGSLAM_TIMEOUT:
                status_array[TAGSLAM_ALIVE] = GREEN
            
            if abs(shortest_angular_distance(euler_from_quaternion([imu_orientation.x, imu_orientation.y, imu_orientation.z, imu_orientation.w])[2], euler_from_quaternion([tagslam_tf.transform.rotation.x, tagslam_tf.transform.rotation.y, tagslam_tf.transform.rotation.z, tagslam_tf.transform.rotation.w])[2])) < IMU_ZERO_ANGLE_THRESHOLD:
                status_array[IMU_CORRECT] = GREEN
        except:
            pass

        try:
            map_to_base_link_tf: TransformStamped = tfBuffer.lookup_transform("map", "base_link", rospy.Time())
            yaw_on_field = euler_from_quaternion([imu_orientation.x, imu_orientation.y, imu_orientation.z, imu_orientation.w])[2]

            # nesting instead of `and` for readability
            if (rospy.Time().to_sec() - map_to_base_link_tf.header.stamp.to_sec()) < MAP_TO_BASE_LINK_TIMEOUT:
                # print(f"rot: {abs(shortest_angular_distance(yaw_on_field, wanted_r))}, hypot: {hypot(map_to_base_link_tf.transform.translation.x - wanted_x, map_to_base_link_tf.transform.translation.y - wanted_y)}")
                if abs(shortest_angular_distance(yaw_on_field, wanted_r)) < AUTO_ANGLE_THRESHOLD:
                    status_array[AUTO_ROTATION_CLOSE] = GREEN
                
                if hypot(map_to_base_link_tf.transform.translation.x - wanted_x, map_to_base_link_tf.transform.translation.y - wanted_y) < AUTO_POSITION_THRESHOLD:
                    status_array[AUTO_POSITION_CLOSE] = GREEN
        except:
            pass

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
        try:
            colour_client(led_arr_msg)
        except:
            rospy.logerr_throttle(1.0, "2025_candle_prematch: unable to call LED service?")
        for idx in [DOT9V0, DOT9V1, DOT10V0, DOT10V1, COLOR_RAW_MATCH_DATA, COLOR_MATCH_DATA, COLOR_MATCH_DATA_2, TAGSLAM_ALIVE, IMU_CORRECT, AUTO_POSITION_CLOSE, AUTO_ROTATION_CLOSE, ELEVATOR_AVOID_TRIGGERED, HAVE_CORAL, CMD_VEL_ZERO, CORRECT_AUTO]:
            blink_idx(idx, ORANGE)
        blink_idx(HEARTBEAT, (255, 255, 255))
        if is_enabled:
            break
        r.sleep()