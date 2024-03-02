#! /usr/bin/env python3

import rospy
from behavior_actions.msg import Intaking2024Action, Intaking2024Goal, Intaking2024Feedback, Intaking2024Result

from talon_controller_msgs.srv import Command, CommandRequest, CommandResponse
from talon_state_msgs.msg import TalonFXProState
from sensor_msgs.msg import JointState

from ddynamic_reconfigure_python.ddynamic_reconfigure import DDynamicReconfigure
# Brings in the SimpleActionClient
import actionlib
from std_msgs.msg import Float64
from norfair_ros.msg import Detections, Detection
from frc_msgs.srv import RumbleCommand, RumbleCommandRequest, RumbleCommandResponse
import math
import time

class Rumble2024Server():

    def __init__(self, name):        
        #self.arm_client = actionlib.SimpleActionClient('/arm/move_arm_server_2024', Arm2024Action)
        #rospy.loginfo("2024_intaking_server: waiting for arm server")
        #self.arm_client.wait_for_server()
        rospy.loginfo("Rumble server starting")

        #self.shooter_pos_sub = rospy.Subscriber("/", TalonFXProState, shooter_pivot_callback)
        self.rumble_srv = rospy.ServiceProxy("/frcrobot_rio/rumble_controller/command", RumbleCommand)
        
        self.intaking_current = 0.0
        self.intaking_talon_idx = None
        self.talonfxpro_sub = rospy.Subscriber('/frcrobot_jetson/talonfxpro_states', TalonFXProState, self.talonfxpro_states_cb)
        #self.norfair_sub = rospy.Subscriber('/norfair/output', Detections, self.notes_callback)
        self.limit_switch_sub = rospy.Subscriber("/frcrobot_rio/joint_states", JointState, self.limit_switch_cb)

        self.notes_max_distance = rospy.get_param("note_distance_away")
        self.intake_limit_switch_name = rospy.get_param("intake_limit_switch_name")
        self.rumble_value = rospy.get_param("rumble_on_note")

        self.rumble = rospy.Timer(rospy.Duration(1.0/20.0), self.rumble_loop)
        self.closest_note = 900
        self.time_touched_note = rospy.Time.now()
        self.touched_note = False
        self.note_left = False

        self.already_touched_note = False


    def talonfxpro_states_cb(self, states: TalonFXProState):
        #rospy.loginfo_throttle(5, "Intaking node recived talonfx pro states")
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

    def limit_switch_cb(self, data):
        #rospy.loginfo_throttle(1, "2024_rumble_server: limit switch callback")
        # check claw switch
        if self.intake_limit_switch_name in data.name:
            if self.touched_note == False:
                #rospy.loginfo(f'2024_rumble_server: {self.intake_limit_switch_name} found')
                self.touched_note = data.position[data.name.index(self.intake_limit_switch_name)]
                self.time_touched_note = rospy.Time.now()
                self.note_has_left = False
            elif self.touched_note == True:
                if not data.position[data.name.index(self.intake_limit_switch_name)]:
                    self.note_left = True
                    self.touched_note = False
        else:
            rospy.logerr_throttle(1.0, f'2024_rumble_server: {self.intake_limit_switch_name} not found')
            pass

    # runs at some hz
    def rumble_loop(self, event):
        #rospy.loginfo_throttle(1, "Rumble running")
        rumble_srv = RumbleCommandRequest()
        rumble_srv.left = 0
        rumble_srv.right = 0

        # TODO add for next event
        # if self.closest_note < self.notes_max_distance:
        #     rumble_srv.left = 10000 # slight rumble if we can see a note
        #     rumble_srv.right = 10000

        if self.touched_note and not self.already_touched_note:
            rospy.logerr_throttle(1, "2024 rumble server: touched note")
            # TODO make this a parameter
            # note should be gone after 1 second
            rumble_srv.left = self.rumble_value
            rumble_srv.right = self.rumble_value
            self.rumble_srv.call(rumble_srv)
            time.sleep(1)
            rospy.loginfo("Sleep done touched note")
            self.touched_note = False
            self.already_touched_note = True

        if self.note_left:
            rospy.loginfo("Note left")
            rumble_srv.left = self.rumble_value
            rumble_srv.right = self.rumble_value
            self.rumble_srv.call(rumble_srv)
            time.sleep(1)
            rospy.loginfo("Sleep done note left")
            self.note_left = False
            self.note_has_left = True
            self.already_touched_note = False
        self.rumble_srv.call(rumble_srv)
        
        

       
if __name__ == '__main__':
    rospy.init_node('intaking_server_2024')
    server = Rumble2024Server(rospy.get_name())
    rospy.spin()