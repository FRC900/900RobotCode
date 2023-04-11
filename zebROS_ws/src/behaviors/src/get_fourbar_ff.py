#!/usr/bin/env python3

# Script to read steady state motor output of the 4bar in a sweep of positions
# Used to generate the interpolating map values for the 4bar gravity feed forward config
import rospy

from talon_state_msgs.msg import TalonState
from controllers_2023_msgs.srv import FourBarSrv


global talon_position
global talon_motor_output

def callback(talon_state_msg : TalonState):
    i = talon_state_msg.name.index('four_bar')
    global talon_position
    global talon_motor_output

    talon_position = talon_state_msg.position[i]
    talon_motor_output = talon_state_msg.motor_output_percent[i]


def main():

    rospy.init_node('get_fourbar_ff')
    rospy.Subscriber("/frcrobot_jetson/talon_states", TalonState, callback)

    four_bar_target = 0.0
    four_bar_service = rospy.ServiceProxy('/frcrobot_jetson/four_bar_controller_2023/four_bar_service', FourBarSrv)
    while (not rospy.is_shutdown()) and (four_bar_target < 1.8):

        four_bar_service(0.2)
        rospy.sleep(2)
        four_bar_service(four_bar_target)
        rospy.sleep(3)
        print(f"\t - [{talon_position}, {talon_motor_output}]")
        four_bar_target += 0.05

if __name__ == "__main__":
    main()