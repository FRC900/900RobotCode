#! /usr/bin/env python3

import rospy
import actionlib
import std_msgs.msg

from talon_state_msgs.msg import TalonFXProState
from behavior_actions.msg import Shooter2024Action, Shooter2024Goal, Shooter2024Feedback, Shooter2024Result




global left_joint_velocity
global right_joint_velocity

global left_joint_index
global right_joint_index

left_joint_index = None
right_joint_index = None
left_joint_velocity = 0.0
right_joint_velocity = 0.0

def callback(data):
    global left_joint_velocity
    global right_joint_velocity
    global left_joint_index
    global right_joint_index

    if (left_joint_index == None) or (right_joint_index == None):
        for i in range(len(data.name)):
            #rospy.loginfo(data.name[i])
            if (data.name[i] == "left_shooter_joint"): 
                left_joint_velocity = data.velocity[i]
                left_joint_index = i
            elif (data.name[i] == "right_shooter_joint"):
                right_joint_velocity = data.velocity[i]  
                rospy.loginfo(i)
                right_joint_index = i
        return
    else:
        left_joint_velocity = data.velocity[left_joint_index]
        right_joint_velocity = data.velocity[right_joint_index]
        


class ShooterServer2024:
    _result = Shooter2024Result()
    _feedback = Shooter2024Feedback()
    def __init__(self):
        self.server = actionlib.SimpleActionServer('set_shooter_speed', Shooter2024Action, self.execute_cb, auto_start = False)
     

        #figure out how client stuff works
        rospy.Subscriber('/frcrobot_jetson/talonfxpro_states', TalonFXProState, callback)
        #subscribing here so that we can figure out the actual speed of said motor at a given time, talonfxpro_states gives us these values
        self.left_joint_shooter_pub = rospy.Publisher("/frcrobot_jetson/left_shooter_voltage_velocity_controller/command", std_msgs.msg.Float64, queue_size=1)
        self.right_joint_shooter_pub = rospy.Publisher("/frcrobot_jetson/right_shooter_voltage_velocity_controller/command", std_msgs.msg.Float64, queue_size=1)
   
        self.server.start()
        
    def execute_cb(self, goal):
        global left_joint_velocity
        global right_joint_velocity
        
        r = rospy.Rate(50)
        initial_left_speed = left_joint_velocity
        initial_right_speed = right_joint_velocity

        self.left_joint_shooter_pub.publish(std_msgs.msg.Float64(goal.left_shooter_speed))
        self.right_joint_shooter_pub.publish(std_msgs.msg.Float64(goal.right_shooter_speed))

     
        
        while True:
            if rospy.is_shutdown():
                break
            r.sleep()

            if goal.left_shooter_speed == 0.0:
                self.left_joint_shooter_pub.publish(std_msgs.msg.Float64(goal.left_shooter_speed))
                left_percent_difference = 0.0
                self._result.success = True
                self._feedback.left_percent_complete = 100.0
                self._feedback.is_shooting_at_speed = True
                self.server.publish_feedback(self._feedback)
                r.sleep()
            
            if goal.right_shooter_speed == 0.0:
                self.right_joint_shooter_pub.publish(std_msgs.msg.Float64(goal.right_shooter_speed))
                right_percent_difference = 0.0
                self._result.success = True
                self._feedback.right_percent_complete = 100.0
                self._feedback.is_shooting_at_speed = True
                self.server.publish_feedback(self._feedback)
                r.sleep()                

            if goal.left_shooter_speed != 0.0:
                self._feedback.left_percent_complete = (((left_joint_velocity - initial_left_speed) / (goal.left_shooter_speed - initial_left_speed))) * 100
                left_percent_difference = (((abs(left_joint_velocity - goal.left_shooter_speed)) / ((left_joint_velocity + goal.left_shooter_speed))) / 2) * 100

            if goal.right_shooter_speed != 0.0:
                self._feedback.right_percent_complete = (((right_joint_velocity - initial_right_speed) / (goal.right_shooter_speed - initial_right_speed))) * 100
                right_percent_difference = (((abs(right_joint_velocity - goal.right_shooter_speed)) / ((right_joint_velocity + goal.right_shooter_speed))) / 2) * 100

            tolerance = .9
            self.server.publish_feedback(self._feedback)

            if self.server.is_preempt_requested():
               self.left_joint_shooter_pub.publish(std_msgs.msg.Float64(goal.left_shooter_speed))
               self.right_joint_shooter_pub.publish(std_msgs.msg.Float64(goal.right_shooter_speed))
               self.server.set_preempted()
               break
            

            elif ((left_percent_difference < tolerance) and (right_percent_difference < tolerance)):
                self._result.success = True
                self._feedback.left_percent_complete = 100.0
                self._feedback.right_percent_complete = 100.0
                self._feedback.is_shooting_at_speed = True
                self.server.publish_feedback(self._feedback)
                r.sleep()
                self.server.set_succeeded(self._result)
                break

if __name__ == '__main__':
    rospy.init_node('shooter_server_2024')

    server = ShooterServer2024()
    rospy.spin()
