#!/usr/bin/env python3
import rospy
import actionlib
import std_msgs.msg

from talon_state_msgs.msg import TalonFXProState
from talon_controller_msgs.srv import Command, CommandRequest, CommandResponse
from behavior_actions.msg import Shooter2024Action, Shooter2024Goal, Shooter2024Feedback, Shooter2024Result

class ShooterServer2024:
    _result = Shooter2024Result()
    _feedback = Shooter2024Feedback()
    tolerance = 0.9 # percent

    top_left_joint_index = None
    top_right_joint_index = None
    bottom_left_joint_index = None
    bottom_right_joint_index = None

    top_left_joint_velocity = 0.0
    top_right_joint_velocity = 0.0
    bottom_left_joint_velocity = 0.0
    bottom_right_joint_velocity = 0.0

    def __init__(self):
        self.server = actionlib.SimpleActionServer('set_shooter_speed', Shooter2024Action, self.execute_cb, auto_start = False)
        self.tolerance = rospy.get_param("tolerance")

        self.talon_states_sub = rospy.Subscriber('/frcrobot_jetson/talonfxpro_states', TalonFXProState, self.callback, tcp_nodelay=True, queue_size=1) # subscribing here so that we can figure out the actual speed of said motor at a given time, talonfxpro_states gives us these values
        
        self.top_left_client = rospy.ServiceProxy("/frcrobot_jetson/top_left_shooter_controller/command", Command)
        self.top_right_client = rospy.ServiceProxy("/frcrobot_jetson/top_right_shooter_controller/command", Command)
        self.bottom_left_client = rospy.ServiceProxy("/frcrobot_jetson/bottom_left_shooter_controller/command", Command)
        self.bottom_right_client = rospy.ServiceProxy("/frcrobot_jetson/bottom_right_shooter_controller/command", Command)

        rospy.loginfo("2024_shooter_server: waiting for shooter controller services")
        self.top_left_client.wait_for_service()
        self.top_right_client.wait_for_service()
        self.bottom_left_client.wait_for_service()
        self.bottom_right_client.wait_for_service()


        rospy.loginfo("2024_shooter_server: starting up")

        self.server.start()

    def callback(self, data):
        if self.top_left_joint_index is None or self.top_right_joint_index is None or self.bottom_left_joint_index is None or self.bottom_right_joint_index is None:
            for i in range(len(data.name)):
                if (data.name[i] == "top_left_shooter_joint"): 
                    self.top_left_joint_velocity = data.velocity[i]
                    self.top_left_joint_index = i
                elif (data.name[i] == "top_right_shooter_joint"):
                    self.top_right_joint_velocity = data.velocity[i]
                    self.top_right_joint_index = i
                elif (data.name[i] == "bottom_left_shooter_joint"): 
                    self.bottom_left_joint_velocity = data.velocity[i]
                    self.bottom_left_joint_index = i
                elif (data.name[i] == "bottom_right_shooter_joint"):
                    self.bottom_right_joint_velocity = data.velocity[i]
                    self.bottom_right_joint_index = i
            return
        else:
            self.top_left_joint_velocity = data.velocity[self.top_left_joint_index]
            self.top_right_joint_velocity = data.velocity[self.top_right_joint_index]
            self.bottom_left_joint_velocity = data.velocity[self.bottom_left_joint_index]
            self.bottom_right_joint_velocity = data.velocity[self.bottom_right_joint_index]
        
    def execute_cb(self, goal: Shooter2024Goal):
        r = rospy.Rate(60)
        initial_top_left_speed = self.top_left_joint_velocity
        initial_top_right_speed = self.top_right_joint_velocity
        initial_bottom_left_speed = self.bottom_left_joint_velocity
        initial_bottom_right_speed = self.bottom_right_joint_velocity

        self.top_left_client.call(CommandRequest(command=goal.top_left_speed))
        self.top_right_client.call(CommandRequest(command=goal.top_right_speed))
        self.bottom_left_client.call(CommandRequest(command=goal.bottom_left_speed))
        self.bottom_right_client.call(CommandRequest(command=goal.bottom_right_speed))
        
        while not rospy.is_shutdown():
            if goal.top_left_speed == 0.0 or goal.top_right_speed == 0.0 or goal.bottom_left_speed == 0.0 or goal.bottom_right_speed == 0.0:
                # if one is zero, set all to zero and return
                self.top_left_client.call(CommandRequest(command=0))
                self.top_right_client.call(CommandRequest(command=0))
                self.bottom_left_client.call(CommandRequest(command=0))
                self.bottom_right_client.call(CommandRequest(command=0))
                self._result.success = True
                self._feedback.top_left_percent_complete = 100.0
                self._feedback.top_right_percent_complete = 100.0
                self._feedback.bottom_left_percent_complete = 100.0
                self._feedback.bottom_right_percent_complete = 100.0
                self._feedback.is_shooting_at_speed = True
                self.server.publish_feedback(self._feedback)
                rospy.loginfo("SHOOTER SERVER, publishing feedback ALL ZEROS")
                r.sleep()
                self.server.set_succeeded(self._result)
                return
        
            else:
                self._feedback.top_left_percent_complete = (((self.top_left_joint_velocity - initial_top_left_speed) / (goal.top_left_speed - initial_top_left_speed))) * 100
                top_left_percent_difference = abs((((abs(self.top_left_joint_velocity - goal.top_left_speed)) / ((self.top_left_joint_velocity + goal.top_left_speed))) / 2) * 100)
                self._feedback.top_right_percent_complete = (((self.top_right_joint_velocity - initial_top_right_speed) / (goal.top_right_speed - initial_top_right_speed))) * 100
                top_right_percent_difference = abs((((abs(self.top_right_joint_velocity - goal.top_right_speed)) / ((self.top_right_joint_velocity + goal.top_right_speed))) / 2) * 100)
                self._feedback.bottom_left_percent_complete = (((self.bottom_left_joint_velocity - initial_bottom_left_speed) / (goal.bottom_left_speed - initial_bottom_left_speed))) * 100
                bottom_left_percent_difference = abs((((abs(self.bottom_left_joint_velocity - goal.bottom_left_speed)) / ((self.bottom_left_joint_velocity + goal.bottom_left_speed))) / 2) * 100)
                self._feedback.bottom_right_percent_complete = (((self.bottom_right_joint_velocity - initial_bottom_right_speed) / (goal.bottom_right_speed - initial_bottom_right_speed))) * 100
                bottom_right_percent_difference = abs((((abs(self.bottom_right_joint_velocity - goal.bottom_right_speed)) / ((self.bottom_right_joint_velocity + goal.bottom_right_speed))) / 2) * 100)

            self.server.publish_feedback(self._feedback)

            if self.server.is_preempt_requested() and not goal.leave_spinning:
                rospy.loginfo("2024_shooter_server: preempted")
                self.top_left_client.call(CommandRequest(command=0))
                self.top_right_client.call(CommandRequest(command=0))
                self.bottom_left_client.call(CommandRequest(command=0))
                self.bottom_right_client.call(CommandRequest(command=0))
                self.server.set_preempted()
                return

            if (top_left_percent_difference < self.tolerance) and (top_right_percent_difference < self.tolerance) and (bottom_left_percent_difference < self.tolerance) and (bottom_right_percent_difference < self.tolerance):
                self._result.success = True
                self._feedback.top_left_percent_complete = 100.0
                self._feedback.top_right_percent_complete = 100.0
                self._feedback.bottom_left_percent_complete = 100.0
                self._feedback.bottom_right_percent_complete = 100.0
                self._feedback.is_shooting_at_speed = True
                self.server.publish_feedback(self._feedback)
                rospy.loginfo("SHOOTER SERVER, publishing feedback and setting succeeded")
                r.sleep()
                self.server.set_succeeded(self._result)
                return
            
            r.sleep()

if __name__ == '__main__':
    rospy.init_node('shooter_server_2024')

    server = ShooterServer2024()
    rospy.spin()
