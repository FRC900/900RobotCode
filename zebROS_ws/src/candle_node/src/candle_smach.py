#!/usr/bin/env python3

import rospy
import smach
import smach_ros
import sys
import time
from candle_controller_msgs.srv import Colour, ColourRequest, Animation, AnimationRequest
from frc_msgs.msg import MatchSpecificData
from behavior_actions.msg import AutoMode
from behavior_actions.msg import AutoAlignSpeaker
from frc_msgs.msg import MatchSpecificData
from sensor_msgs.msg import JointState



def match_data_callback(msg):
    global is_disabled
    is_disabled = msg.Disabled
    global is_auto
    is_auto = msg.Autonomous
    global team_color

    team_color = [252, 186, 3]

   

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
    rospy.wait_for_service('/frcrobot_jetson/candle_controller/animation')
    try:
        animation_client = rospy.ServiceProxy('/frcrobot_jetson/candle_controller/colour', Animation)
        animation_client(animation)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def limit_swtich_callback(data):
    global has_note
    preshooter_limit_switch = "preshooter_limit_switch"
    if preshooter_limit_switch in data.name:
        preshooter_switch = data.position[data.name.index(preshooter_limit_switch)]
        if preshooter_switch == 1:
            has_note = True
        else:
            has_note = False


class head_state(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['disabled', 'auto_state', 'in_range', 'not_in_range', 'no_note', 'blue_test_state'])

    def execute(self, userdata):
        #if is_disabled or is_auto: # Do we want something different if god forbid we disable during the match?
        #    if auto_mode > 4:
        #        return 'disabled'
        #    return 'auto_state'
        if is_disabled:
            return 'disabled'
        #elif is_auto:
        #    return 'auto_state'

        elif has_note:
                send_colour(78, 16, 133)
                if (in_range == True):
                    return 'in_range'
                elif (in_range != True):
                    return 'not_in_range'

        return 'no_note'
        #else:
        #return 'blue_test_state'



    
class auto_state(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['teleop'])

    def execute(self, userdata):
        rospy.loginfo('Executing state auto_state (rainbow), prepare for enlightenment')
        #send_animation(1, 0, 6, 3)
        send_colour(78, 16, 133)
        #while (is_auto): # A bit fishy
        #    r.sleep()
        return 'teleop'

class disabled(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['teleop'])

    def execute(self, userdata):
        rospy.loginfo('Executing state disabled, white light :')
        send_colour(255, 255, 255) # white :)
        #while (is_disabled): # A bit fishy
        #    r.sleep()
        return 'teleop'



class blue_test_state(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['head_state'])
    
    def execute(self, userdata):
        rospy.loginfo('In blue tests teat mahcine thing')
        send_colour(0, 0, 255)
        return 'head_state'

class ready_to_shoot(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['lost_note', 'left_range'])
        
    def execute(self, userdata):
        rospy.loginfo('Executing state ready_to_shoot (red light), piece acquired')
        send_colour(189, 21, 46) # Pink
        #while has_note and in_range: # Half-real
        #    r.sleep()
        if not has_note: # Not real
            return 'lost_note'
        return 'left_range'

class out_of_range(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['lost_note', 'entered_range'])
        
    def execute(self, userdata):
        rospy.loginfo('Executing state out_of_range (green light)')
        send_colour(0, 255, 0) # Green
        #while has_note and (not in_range): # Half-real
        #    r.sleep()
        if not has_note: # Not real
            return 'lost_note'
        return 'entered_range'

class noteless(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['got_note'])
        
    def execute(self, userdata):
        rospy.loginfo('Executing state noteless (yellow_light)')
        send_colour(252, 186, 3)
        #while not has_note: # Not real
        #    r.sleep()
        return 'got_note'

if __name__ == '__main__':
    rospy.init_node('leds_state_machine')
    r = rospy.Rate(10)#10 hz
    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['termination'])

    # These are not being used rn
    # sm.userdata.is_disabled = True
    # sm.userdata.is_auto = False
    # sm.userdata.auto_mode = 1
    # sm.userdata.has_note = False
    # sm.userdata.in_range = False

    is_disabled = False
    is_auto = False
    
    auto_mode = 1
    has_note = False
    in_range = False
    #shooting_distance = rospy.get_param("effective_shooting_range")

    rospy.Subscriber("/bag/match_data", MatchSpecificData, match_data_callback)
    rospy.Subscriber("/auto/auto_mode", AutoMode, auto_mode_callback)
    rospy.Subscriber("/speaker_align/dist_and_ang", AutoAlignSpeaker, distance_callback)
    rospy.Subscriber("/bag/joint_states", JointState, limit_swtich_callback)

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('head_state', head_state(), transitions={'disabled': 'disabled', 'auto_state': 'auto_state', 'in_range': 'ready_to_shoot', 'not_in_range': 'out_of_range', 'no_note': 'noteless', 'blue_test_state':'blue_test_state'})

        smach.StateMachine.add('blue_test_state', blue_test_state(), transitions={'head_state':'head_state'})

        smach.StateMachine.add('disabled', disabled(), transitions={'teleop': 'head_state'})
        smach.StateMachine.add('auto_state', auto_state(), transitions={'teleop': 'head_state'})

        smach.StateMachine.add('ready_to_shoot', ready_to_shoot(), transitions={'lost_note': 'noteless', 'left_range': 'out_of_range'})
        smach.StateMachine.add('out_of_range', out_of_range(), transitions={'lost_note': 'noteless', 'entered_range': 'ready_to_shoot'})
        smach.StateMachine.add('noteless', noteless(), transitions={'got_note': 'head_state'})
    # Execute SMACH plan
    outcome = sm.execute()

