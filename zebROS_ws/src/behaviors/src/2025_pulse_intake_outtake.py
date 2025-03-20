#!/usr/bin/env python3

import rospy
import actionlib
from sensor_msgs.msg import JointState
from behavior_actions.msg import Intaking2025Goal, Intaking2025Feedback, Intaking2025Result, Intaking2025Action
from behavior_actions.msg import Elevater2025Goal, Elevater2025Feedback, Elevater2025Result, Elevater2025Action
from behavior_actions.msg import Roller2025Goal, Roller2025Feedback, Roller2025Result, Roller2025Action
from behavior_actions.msg import PulseOuttake2025Goal, PulseOuttake2025Feedback, PulseOuttake2025Result, PulseOuttake2025Action
from talon_controller_msgs.srv import Command, CommandRequest

# just automates the process of intaking and outtaking when there is a jam

class PulseServer(object):
    def __init__(self, name):
        self.result = PulseOuttake2025Result()
        self.feedback = PulseOuttake2025Feedback()

        self.roller_client = actionlib.SimpleActionClient('/roller/roller_server_2025', Roller2025Action)
        rospy.loginfo("Found elevater server")
        
        self.intake_ac_client = actionlib.SimpleActionClient('/intaking/intaking_server_2025', Intaking2025Action)

        self.intake_srv_client = rospy.ServiceProxy(f"/frcrobot_jetson/intake_controller/command", Command)
        rospy.loginfo(f"Waiting for intaking service at /frcrobot_jetson/intake_controller/command")
        self.intake_srv_client.wait_for_service()

        self.server = actionlib.SimpleActionServer(name, PulseOuttake2025Action, execute_cb=self.execute_cb, auto_start = False)
        self.outtake_time = rospy.get_param("outtake_time")
        #self.switch_sub = rospy.Subscriber("/frcrobot_rio/joint_states", JointState, self.callback, tcp_nodelay=True)
        #self.switch_name = rospy.get_param("switch_name")
        #self.switch = 0.0
        
        self.server.start()
        rospy.loginfo("Started pulse actionlib server")

    def preempt_all(self):
        rospy.logwarn("Intaking Server Preempting")
        self.roller_client.cancel_goals_at_and_before_time(rospy.Time.now()) 
        self.elevater_client.cancel_goals_at_and_before_time(rospy.Time.now())
        self.intake_srv_client.call(CommandRequest(0))

    def callback(self, data):
        if self.switch_name in data.name:
            self.switch = data.position[data.name.index(self.switch_name)]
            #rospy.loginfo(f"Found {self.switch_name} with value {self.switch}")
        else:
            rospy.logwarn_throttle(1.0, f'2025_pulse_server: {self.switch_name} not found')
            pass

    def execute_cb(self, goal : PulseOuttake2025Goal):

        r = rospy.Rate(50.0)

        if not self.intake_srv_client.call(CommandRequest(-3)):
            rospy.loginfo("Intake client service call failed")

        rospy.sleep(self.outtake_time)
        rospy.loginfo("2025_pulse_server: outtaking")

        intake_goal = Intaking2025Goal()

        self.intake_ac_client.send_goal(intake_goal)
        rospy.loginfo("2025_pulse_server: sent intaking goal")

        self.result.success = True
        rospy.loginfo("2025_pulse_server: succeeded")
        self.server.set_succeeded(self.result) 
        

if __name__ == '__main__':
    rospy.init_node('2025_pulse_server')
    server = PulseServer(rospy.get_name())
    rospy.spin()