#!/usr/bin/env python3

import rospy
import sys
import time
from candle_controller_msgs.srv import Colour, ColourRequest, Animation, AnimationRequest
from frc_msgs.msg import MatchSpecificData
from behavior_actions.msg import AutoMode, AlignAndPlace2025ActionFeedback
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from talon_state_msgs.msg import TalonFXProState

# We want a list of priorities for what LED stuff to do
# 1. If we're in autonomous, we want to turn the LEDs rainbow (works)
# 2. Drive to object is running, we want to turn the LEDs rainbow (server problem)
# 3. If we're autoaligning, we want to turn the LEDs white (works)
# 4. If we are intaking, we want to turn the LEDs yellow (works)
# 5. If we have received coral, we want to flash the LEDs green (works)
# 6. If we're in teleop and none of the above apply, make the LEDs our alliance color (works)
# Maybe add one for error?

team_color = [0, 0, 0]

class State:
    def __init__(self, name, condition, action):
        self.name = name
        self.condition = condition # a function
        self.action = action
        self.currently_running = False
    
    def run(self):
        if self.condition() and not self.currently_running:
            rospy.loginfo(f"Entering state {self.name}")
            self.action()
            self.currently_running = True
        elif not self.condition():
            self.currently_running = False
    
    def stop(self):
        self.currently_running = False

last_drive_obj_time = None

def is_drive_objet():
    global last_drive_obj_time
    if rospy.Time.now() - last_drive_obj_time < rospy.Duration(0.2):
        return True
    else:
        return False

states = [
    State("drive_to_object", is_drive_objet, lambda: send_animation(0.75, AnimationRequest.ANIMATION_TYPE_RAINBOW, 0, 0, 0, 0, 0)),
    State("autonomous", lambda: is_auto, lambda: send_animation(0.75, AnimationRequest.ANIMATION_TYPE_RAINBOW, 0, 0, 0, 0, 0)),
    State("alinging_and_placing", lambda: aligning, lambda: send_colour(255,255,255)),
    State("has_coral", lambda: has_coral, lambda: send_colour(0,255,0)),
    State("intaking_coral", lambda: intaking, lambda: send_animation(0.75, AnimationRequest.ANIMATION_TYPE_STROBE, 255, 255, 0, 0, 0)),
    State("teleop", lambda: True, lambda: send_colour(*team_color))
]

def align_and_place_callback(msg):
    global aligning
    aligning = True

def talon_state_callback(msg: TalonFXProState):
    global intaking
    global roller
    intaking = abs(msg.control_output[msg.name.index("intake")]) > 0
    roller = abs(msg.control_output[msg.name.index("roller")]) > 0

def match_data_callback(msg: MatchSpecificData):
    global is_disabled
    is_disabled = msg.Disabled
    global is_auto
    is_auto = msg.Autonomous
    global team_color
    if msg.allianceColor == msg.ALLIANCE_COLOR_BLUE:
        team_color = [0,0,255]
    elif msg.allianceColor == msg.ALLIANCE_COLOR_RED:
        team_color = [255,0,0]

def drive_object_callback(msg):
    global last_drive_obj_time
    last_drive_obj_time = rospy.Time.now()

def auto_mode_callback(msg):
    global auto_mode
    auto_mode = msg.auto_mode

def send_colour(r_col, g_col, b_col):
    colour = ColourRequest()
    # Start/counts should be edited to match the real robot
    colour.start = 8
    colour.count = 18
    colour.red = r_col
    colour.green = g_col
    colour.blue = b_col
    colour.white = 0
    rospy.loginfo(f"Sending color to candle controller with red {r_col}, green {g_col}, blue {b_col}")
    rospy.wait_for_service('/frcrobot_jetson/candle_controller/colour')
    try:
        colour_client = rospy.ServiceProxy('/frcrobot_jetson/candle_controller/colour', Colour)
        colour_client(colour)
    except rospy.ServiceException as e:
        rospy.loginfo("Service call failed: %s"%e)

def send_animation(speed, animation_type, red, green, blue, white, direction):
    animation = AnimationRequest()
    animation.speed = speed
    animation.start = 8
    animation.count = 18
    animation.animation_type = animation_type
    animation.red = red
    animation.green = green
    animation.blue = blue
    animation.white = white
    animation.direction = direction
    animation.brightness = 0.75
    animation.reversed = False
    animation.param4 = 0
    animation.param5 = 0
    rospy.loginfo(f"Sending animation {animation_type} to candle controller with speed {speed}, red {red}, green {green}, blue {blue}, white {white}, direction {direction}")
    rospy.wait_for_service('/frcrobot_jetson/candle_controller/animation')
    try:
        animation_client = rospy.ServiceProxy('/frcrobot_jetson/candle_controller/animation', Animation)
        animation_client(animation)
    except rospy.ServiceException as e:
        rospy.loginfo("Service call failed: %s"%e)

def limit_switch_callback(data):
    global has_coral
    if "prescoring_limit_switch" in data.name:
        prescoring_switch = data.position[data.name.index("prescoring_limit_switch")]
        has_coral = prescoring_switch

if __name__ == '__main__':
    rospy.init_node('leds_state_machine')
    r = rospy.Rate(60) # 60 Hz

    last_drive_obj_time = rospy.Time.now()

    is_disabled = False
    is_auto = False
    
    auto_mode = 1
    has_coral = False
    in_range = False
    ready_to_intake = False
    intaking = False
    aligning = False
    roller = False

    rospy.Subscriber("/frcrobot_rio/match_data", MatchSpecificData, match_data_callback)
    rospy.Subscriber("/auto/auto_mode", AutoMode, auto_mode_callback)
    rospy.Subscriber("/frcrobot_rio/joint_states", JointState, limit_switch_callback)
    rospy.Subscriber("/auto_align/cmd_vel", Twist, drive_object_callback)
    rospy.Subscriber("/frcrobot_jetson/talonfxpro_states", TalonFXProState, talon_state_callback)
    rospy.Subscriber("/align_and_place/alignandplaceing_server/feedback", AlignAndPlace2025ActionFeedback, align_and_place_callback)

    while not rospy.is_shutdown():
        got_valid_state = False
        for state in states:
            if got_valid_state:
                state.stop()
            else:
                state.run()
                if state.currently_running:
                    got_valid_state = True
        r.sleep()
        
    rospy.spin()
