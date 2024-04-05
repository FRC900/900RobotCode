#!/usr/bin/env python3

import rospy
import sys
import time
from candle_controller_msgs.srv import Colour, ColourRequest, Animation, AnimationRequest
from frc_msgs.msg import MatchSpecificData
from behavior_actions.msg import AutoMode
from behavior_actions.msg import AutoAlignSpeaker
from frc_msgs.msg import MatchSpecificData
from sensor_msgs.msg import JointState
from norfair_ros.msg import Detections

# We want a list of priorities for what LED stuff to do
# 1. If we're in autonomous, we want to turn the LEDs rainbow
# 2. If we're within shooting range, we want to turn the LEDs green
# 3. If we have a note, we want to turn the LEDs orange
# 4. If we see a note, we want to turn the LEDs white with a "fire" animation
# 5. If we're in teleop and none of the above apply, make the LEDs our alliance color

team_color = [0, 0, 0]

class State:
    def __init__(self, name, condition, action):
        self.name = name
        self.condition = condition # a function
        self.action = action
        self.currently_running = False
    
    def run(self):
        if self.condition() and not self.currently_running:
            print(f"Entering state {self.name}")
            self.action()
            self.currently_running = True
        elif not self.condition():
            self.currently_running = False
    
    def stop(self):
        self.currently_running = False

states = [
    State("autonomous", lambda: is_auto, lambda: send_animation(0.5, 0, 15, 0)),
    State("in_range", lambda: has_note and in_range, lambda: send_colour(0, 255, 0)),
    State("has_note", lambda: has_note, lambda: send_colour(255, 165, 0)),
    State("can_see_note", lambda: can_see_note, lambda: send_animation(0.5, 0, 15, 1)),
    State("teleop", lambda: True, lambda: send_colour(*team_color))
]

can_see_note = False
def norfair_callback(msg: Detections):
    global can_see_note
    can_see_note = "note" in map(lambda d: d.label, msg.detections)

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

def auto_mode_callback(msg):
   global auto_mode
   auto_mode = msg.auto_mode

def distance_callback(msg):
   global in_range
   shooting_distance = 4.5
   if (msg.distance < shooting_distance):
    in_range = True
   else:
    in_range = False

def send_colour(r_col, g_col, b_col):
    colour = ColourRequest()
    # Start/counts should be edited to match the real robot
    colour.start = 9
    colour.count = 15
    colour.red = r_col
    colour.green = g_col
    colour.blue = b_col
    colour.white = 0
    print(f"Sending colour to candle controller with red {r_col}, green {g_col}, blue {b_col}")
    rospy.wait_for_service('/frcrobot_jetson/candle_controller/colour')
    try:
        colour_client = rospy.ServiceProxy('/frcrobot_jetson/candle_controller/colour', Colour)
        colour_client(colour)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def send_animation(speed, start, count, animation_type):
    animation = AnimationRequest()
    animation.speed = speed
    animation.start = start
    animation.count = count
    animation.animation_type = animation_type
    animation.red = 0
    animation.green = 0
    animation.blue = 0
    animation.white = 0
    animation.direction = 0
    animation.brightness = 0.75
    animation.reversed = False
    animation.param4 = 0
    animation.param5 = 0
    print(f"Sending animation {animation_type} to candle controller with speed {speed}, start {start}, count {count} and brightness {animation.brightness}")
    rospy.wait_for_service('/frcrobot_jetson/candle_controller/animation')
    try:
        animation_client = rospy.ServiceProxy('/frcrobot_jetson/candle_controller/colour', Animation)
        animation_client(animation)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def limit_switch_callback(data):
    global has_note
    preshooter_limit_switch = "preshooter_limit_switch"
    if preshooter_limit_switch in data.name:
        preshooter_switch = data.position[data.name.index(preshooter_limit_switch)]
        if preshooter_switch == 1:
            has_note = True
        else:
            has_note = False

if __name__ == '__main__':
    rospy.init_node('leds_state_machine')
    r = rospy.Rate(60) # 60 Hz

    is_disabled = False
    is_auto = False
    
    auto_mode = 1
    has_note = False
    in_range = False
    #shooting_distance = rospy.get_param("effective_shooting_range")

    rospy.Subscriber("/frcrobot_rio/match_data", MatchSpecificData, match_data_callback)
    rospy.Subscriber("/auto/auto_mode", AutoMode, auto_mode_callback)
    rospy.Subscriber("/speaker_align/dist_and_ang", AutoAlignSpeaker, distance_callback)
    rospy.Subscriber("/frcrobot_rio/joint_states", JointState, limit_switch_callback)
    rospy.Subscriber("/norfair/output", Detections, norfair_callback)

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

