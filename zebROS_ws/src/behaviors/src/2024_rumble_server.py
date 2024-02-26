#! /usr/bin/env python3

import rospy
from behavior_actions.msg import Intaking2024Action, Intaking2024Goal, Intaking2024Feedback, Intaking2024Result

from talon_controller_msgs.srv import Command, CommandRequest, CommandResponse
from talon_state_msgs.msg import TalonFXProState

from ddynamic_reconfigure_python.ddynamic_reconfigure import DDynamicReconfigure
# Brings in the SimpleActionClient
import actionlib
from std_msgs.msg import Float64
from norfair_ros.msg import Detections, Detection
from frc_msgs.srv import RumbleCommand, RumbleCommandRequest, RumbleCommandResponse
import math

class Rumble2024Server():

    def __init__(self, name):        
        #self.arm_client = actionlib.SimpleActionClient('/arm/move_arm_server_2024', Arm2024Action)
        #rospy.loginfo("2024_intaking_server: waiting for arm server")
        #self.arm_client.wait_for_server()
        rospy.loginfo("Rumble server starting")

        #self.shooter_pos_sub = rospy.Subscriber("/", TalonFXProState, shooter_pivot_callback)
        self.rumble_srv = rospy.ServiceProxy("/RUMBLETOPIC", RumbleCommand)
        
        self.intaking_current = 0.0
        self.intaking_talon_idx = None
        self.talonfxpro_sub = rospy.Subscriber('/frcrobot_jetson/talonfxpro_states', TalonFXProState, self.talonfxpro_states_cb)
        
        self.norfair_sub = rospy.Subscriber('/norfair/output', Detections, self.notes_callback)
        self.notes_max_distance = rospy.get_param("note_distance_away")

        self.rumble = rospy.Timer(rospy.Duration(1.0/20.0), self.rumble_loop)

    def talonfxpro_states_cb(self, states: TalonFXProState):
        rospy.loginfo_throttle(5, "Intaking node recived talonfx pro states")
        if (self.intaking_talon_idx == None):
            for i in range(len(states.name)):
                #rospy.loginfo(data.name[i])
                if (states.name[i] == "intake"): 
                    self.intaking_current = states.torque_current[i]
                    self.intaking_talon_idx = i
                    break
        else:
            self.intaking_current = states.torque_current[self.intaking_talon_idx]

    # recives norfair messages and finds the closest note
    def notes_callback(self, msg: Detections):
        closest_dist = None
        for detection in msg.detections:
            if detection.label != "note":
                continue
            x, y = detection.points[0].point
            dist = math.hypot(x, y)
            if dist < closest_dist:
                closest_dist = dist
            #rospy.loginfo(f"Note detection at {x, y}")
        self.closest_note = closest_dist
    
    def has_game_piece_callback(self, msg):
        pass 

    # runs at some hz
    def rumble_loop(self, event):
        rospy.loginfo("Rumble running")
        rumble_srv = RumbleCommandRequest()
        rumble_srv.left = 0
        rumble_srv.right = 0

        if self.closest_note < self.notes_max_distance:
            rumble_srv.left = 10000 # slight rumble if we can see a note
            rumble_srv.right = 10000

        
         
        self.rumble_srv.call(rumble_srv)



       
if __name__ == '__main__':
    rospy.init_node('intaking_server_2024')
    server = Rumble2024Server(rospy.get_name())
    rospy.spin()