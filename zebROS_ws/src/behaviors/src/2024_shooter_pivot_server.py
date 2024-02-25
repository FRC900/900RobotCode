#! /usr/bin/env python3

import rospy
import actionlib
import std_msgs.msg

from talon_state_msgs.msg import TalonFXProState
from behavior_actions.msg import ShooterPivot2024Action, ShooterPivot2024Goal, ShooterPivot2024Feedback, ShooterPivot2024Result

global motion_magic_value
global motion_magic_value_index


motion_magic_value_index = None
motion_magic_value = 1.0


def callback(data):
    global motion_magic_value
    global motion_magic_value_index
    if (motion_magic_value_index == None):
        for i in range(len(data.name)):
            #rospy.loginfo(data.name[i])
            if (data.name[i] == "shooter_pivot_motionmagic_joint"): 
                motion_magic_value = data.position[i]
                motion_magic_value_index = i
                break
    else:
        motion_magic_value = data.position[motion_magic_value_index]



class ShooterPivotServer2024:
    _result = ShooterPivot2024Result()
    _feedback = ShooterPivot2024Feedback()

    def __init__(self):

        self.server = actionlib.SimpleActionServer('set_shooter_pivot', ShooterPivot2024Action, self.execute_cb, auto_start = False)

        rospy.Subscriber('/frcrobot_jetson/talonfxpro_states', TalonFXProState, callback)
        #subscribing here so that we can figure out the actual speed of said motor at a given time, talonfxpro_states gives us these values
        #maybe we subscibe to the voltage velocity controller instead of hte fx pro

        self.shooter_pivot_pub = rospy.Publisher("/frcrobot_jetson/shooter_pivot_motionmagicvoltage_controller/command", std_msgs.msg.Float64, queue_size=1)
   
        self.server.start()

    def execute_cb(self, goal):
        global motion_magic_value
        r = rospy.Rate(50)

        initial_motion_magic_value = motion_magic_value 
        self.shooter_pivot_pub.publish(std_msgs.msg.Float64(goal.pivot_position))
   
        while True:
            if rospy.is_shutdown():
                break

            if (goal.pivot_position - initial_motion_magic_value) == 0.0: #either a double or a int i'm not entirely sure
                self._feedback.percent_complete = 100.0
            else:
                self._feedback.percent_complete = (((motion_magic_value - initial_motion_magic_value) / (goal.pivot_position - initial_motion_magic_value))) * 100
            percent_difference = (((abs(motion_magic_value - goal.pivot_position)) / ((motion_magic_value + goal.pivot_position))) / 2) * 100
            tolerance = .9
            self.server.publish_feedback(self._feedback)

            if self.server.is_preempt_requested():
                self.shooter_pivot_pub.publish(std_msgs.msg.Float64(motion_magic_value))
                self.server.set_preempted()
                break
             
            elif (percent_difference < tolerance):
                self._result.success = True
                self._feedback.percent_complete = 100.0
                self._feedback.is_at_pivot_position = True
                self.server.publish_feedback(self._feedback)
                self.server.set_succeeded(self._result)
                break
            
            r.sleep()
        

if __name__ == '__main__':
    rospy.init_node('shooter_pivot_server_2024')

    server = ShooterPivotServer2024()
    rospy.spin()
