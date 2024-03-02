#!/usr/bin/env python3

# Script to read steady state motor output of the 4bar in a sweep of positions
# Used to generate the interpolating map values for the 4bar gravity feed forward config
import rospy

from talon_state_msgs.msg import TalonState, TalonFXProState
from controllers_2024_msgs.srv import ShooterPivotSrv


global talon_position
global talon_motor_output

def callback(talon_state_msg : TalonFXProState):
    i = talon_state_msg.name.index('shooter_pivot_motionmagic_joint')
    global talon_position
    global talon_motor_output

    talon_position = talon_state_msg.position[i]
    talon_motor_output = talon_state_msg.motor_voltage[i]


def main():

    rospy.init_node('get_pivot_ff')
    rospy.Subscriber("/frcrobot_jetson/talonfxpro_states", TalonFXProState, callback)

    four_bar_start = 0.5
    four_bar_target =  0.5 #four_bar_start
    four_bar_service = rospy.ServiceProxy('/frcrobot_jetson/shooter_pivot_controller/shooter_pivot_service', ShooterPivotSrv)
    while (not rospy.is_shutdown()) and (four_bar_target <= 1.0):

        four_bar_service(four_bar_start)
        rospy.sleep(2)
        four_bar_service(four_bar_target)
        rospy.sleep(3)
        print(f"\t - [{talon_position}, {talon_motor_output}]")
        four_bar_target += 0.05

if __name__ == "__main__":
    main()