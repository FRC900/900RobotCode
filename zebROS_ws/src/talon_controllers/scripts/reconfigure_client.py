#! /usr/bin/env python3

import rospy
from dynamic_reconfigure.client import Client
from dynamic_reconfigure.msg import ConfigDescription

class Joints:
    speed_option = False
    steering_option = False

joints = Joints()
joints.steering_option = rospy.get_param('/reconfigure_client/steering_option')
joints.speed_option = rospy.get_param('/reconfigure_server/speed_option')


#switch statement poss:
#device = rospy.get_param('/node_name/mode') # node_name/argsname
#
var = 0 
speed_param = 0
steering_param = 0
#steering_param = rospy.get_param('/talon_reconfigure_client_dynamic/steering_option)
#speed_param = rospy.get_param('/talon_reconfigure_client_dynamic/speed_option)

#setting up speed stuff for the speed joints




if joints.speed_option == True:

    speed_fr = Client('/frcrobot_jetson/swerve_drive_controller/fr_drive')
    speed_fl = Client('/frcrobot_jetson/swerve_drive_controller/fl_drive')
    speed_br = Client('/frcrobot_jetson/swerve_drive_controller/br_drive')
    speed_bl = Client('/frcrobot_jetson/swerve_drive_controller/bl_drive')



#setting up steering stuff for hte steering joints
if joints.steering_option == True:
    steering_fr = Client('/frcrobot_jetson/swerve_drive_controller/fr_angle')
    steering_fl = Client('/frcrobot_jetson/swerve_drive_controller/fl_angle')
    steering_br = Client('/frcrobot_jetson/swerve_drive_controller/br_angle')
    steering_bl = Client('/frcrobot_jetson/swerve_drive_controller/bl_angle')



#ishowspeed

def handle_config_changes(config_dict): #callback for the subscriber cases
    if joints.speed_option == True:
        speed_fr.update_configuration(config_dict)
        speed_fl.update_configuration(config_dict)
        speed_br.update_configuration(config_dict)
        speed_bl.update_configuration(config_dict)
    if joints.steering_option == True:
        steering_fr.update_configuration(config_dict)
        steering_fl.update_configuration(config_dict)
        steering_br.update_configuration(config_dict)
        steering_bl.update_configuration(config_dict)


#set of clients motors
# the function is called for each different motor set when needed.





def main():
    
    rospy.init_node('talon_reconfigure_client_dynamic')
    #subscribes to the first parameter rostopic, should appear on rostopic list
    rospy.Subscriber('/talon_reconfigure_server/parameter_updates', ConfigDescription, handle_config_changes)
    #subscribes to the first parameter rostopic, should appear on rostopic list
    rospy.spin()

#ok so, idk why but config description works instead of just config? it works though lmao idk why

main()
