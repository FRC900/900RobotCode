#!/usr/bin/env python3

import rospy
import actionlib
from behavior_actions.msg import Climb2024Action, Climb2024Goal, Climb2024Result, Climb2024Feedback
from talon_state_msgs.msg import TalonFXProState
from controllers_2024_msgs.srv import Climb, ClimbRequest
from behavior_actions.msg import ShooterPivot2024Action, ShooterPivot2024Goal, ShooterPivot2024Result, ShooterPivot2024Feedback
import time

class ClimbAction():
    feedback = Climb2024Feedback()
    result = Climb2024Result()

    def __init__(self, name):
        self.action_name = name
        # using service here so that our calls are guaranteed to arrive
        self.climb_client = rospy.ServiceProxy("/frcrobot_jetson/climb_controller/command", Climb)
        self.climb_client.wait_for_service()
        self.server = actionlib.SimpleActionServer(self.action_name, Climb2024Action, execute_cb=self.execute_cb, auto_start=False)

        self.pivot_client = actionlib.SimpleActionClient('/shooter/set_shooter_pivot', ShooterPivot2024Action)
        rospy.loginfo("2024_climbing_server: waiting for pivot")
        self.pivot_client.wait_for_server()
        rospy.loginfo("2024_climbing_server: done waiting")

        self.extend_speed = rospy.get_param("extend_speed")
        self.extend_acceleration = rospy.get_param("extend_acceleration")

        self.fast_retract_speed = rospy.get_param("fast_retract_speed")
        self.trap_retract_speed = rospy.get_param("trap_retract_speed")

        self.fast_retract_acceleration = rospy.get_param("fast_retract_acceleration")
        self.trap_retract_acceleration = rospy.get_param("trap_retract_acceleration")
        
        self.extend_height = rospy.get_param("extend_height")
        self.trap_height = rospy.get_param("trap_height")
        self.fast_height = rospy.get_param("fast_height")

        self.pivot_angle = rospy.get_param("pivot_angle")
        self.min_pivot_angle = rospy.get_param("min_pivot_angle")

        self.joint_name = rospy.get_param("joint_name")
        self.joint_index = None
        self.joint_position = 0.0
        self.joint_control_position = 0.0

        self.state = Climb2024Feedback.RAISING_ARMS

        self.states = {Climb2024Feedback.RAISING_ARMS: self.raising_arms, Climb2024Feedback.LOWERING_ARMS: self.lowering_arms}

        self.rate = rospy.Rate(rospy.get_param("loop_rate", 50.0))

        self.tolerance = rospy.get_param("tolerance") # positional tolerance for exit

        self.pivot_position = 0.0
        self.pivot_index = None

        self.success = True

        self.talon_state_sub = rospy.Subscriber("/frcrobot_jetson/talonfxpro_states", TalonFXProState, callback=self.talon_state_cb)

        self.server.start()

    def talon_state_cb(self, msg: TalonFXProState):
        if self.joint_index is None:
            for i in range(len(msg.name)):
                if msg.name[i] == self.joint_name:
                    self.joint_index = i
        if self.joint_index is None:
            # if it's still None, we haven't found the joint (which we need to monitor position). exit!
            rospy.logerr(f"2024_climbing_server: joint {self.joint_name} not found. Exiting.")
            exit(-1)
        self.joint_position = msg.position[self.joint_index]
        self.joint_control_position = msg.control_position[self.joint_index]

        if self.pivot_index is None:
            for i in range(len(msg.name)):
                if msg.name[i] == "shooter_pivot_motionmagic_joint":
                    self.pivot_index = i
        if self.pivot_index is None:
            # if it's still None, we haven't found the joint (which we need to monitor position). exit!
            rospy.logerr(f"2024_climbing_server: joint shooter_pivot_motionmagic_joint not found. Exiting.")
            exit(-1)
        self.pivot_position = msg.position[self.pivot_index]

    def cleanup(self, goal: Climb2024Goal):
        rospy.loginfo("2024_climbing_server: cleaning up, setting to zero velocity and resetting state")
        req = ClimbRequest()
        req.velocity = 0.0
        req.use_percent_output = True
        self.state = Climb2024Feedback.RAISING_ARMS - (-1) if goal.mode == goal.REVERSE else 1
        self.climb_client.call(req)

    def raising_arms(self, goal: Climb2024Goal):
        req = ClimbRequest()
        req.position = self.extend_height
        req.velocity = self.extend_speed
        req.acceleration = self.extend_acceleration

        self.climb_client.call(req)

        pivot_goal = ShooterPivot2024Goal()
        pivot_goal.pivot_position = self.pivot_angle
        self.pivot_client.send_goal(pivot_goal)

        while not rospy.is_shutdown():
            if self.server.is_preempt_requested():
                rospy.loginfo("2024_climbing_server: preempted")
                self.cleanup(goal)
                self.server.set_preempted()
                self.success = False
                break
                
            if abs(req.position - self.joint_position) < self.tolerance and self.pivot_position > self.min_pivot_angle:
                rospy.loginfo("2024_climbing_server: got to position, continuing")
                break
            
            self.rate.sleep()

        time.sleep(0.5)

        rospy.loginfo("2024_climbing_server: canceling pivot goal")
        self.pivot_client.cancel_goals_at_and_before_time(rospy.Time.now())

    def lowering_arms(self, goal: Climb2024Goal):
        req = ClimbRequest()
        if goal.mode == goal.TRAP or goal.mode == goal.REVERSE:
            req.position = self.trap_height
            req.velocity = self.trap_retract_speed
            req.acceleration = self.trap_retract_acceleration
        if goal.mode == goal.FAST:
            req.position = self.fast_height
            req.velocity = self.fast_retract_speed
            req.acceleration = self.fast_retract_acceleration

        self.climb_client.call(req)

        while not rospy.is_shutdown():
            if self.server.is_preempt_requested():
                rospy.loginfo("2024_climbing_server: preempted")
                self.cleanup(goal)
                self.server.set_preempted()
                self.success = False
                break
                
            if abs(req.position - self.joint_position) < self.tolerance:
                rospy.loginfo("2024_climbing_server: got to position, continuing")
                break
            
            self.rate.sleep()

    def execute_cb(self, goal: Climb2024Goal):
        self.success = True

        if goal.reset:
            rospy.logwarn("2024_climbing_server: resetting state")
            self.state = Climb2024Feedback.RAISING_ARMS
        
        self.states[self.state](goal) # this is cool :)

        self.state += (-1) if goal.mode == goal.REVERSE else 1
        self.state %= 2

        # 0. raise arms up until hit configurable max
        # 1. drive arms down to a configured height (will be different for trap vs fast climb)

        # Reversing the climb is just following these steps in the opposite order

        self.result.success = self.success
        if self.success:
            rospy.loginfo("2024_climbing_server: succeeded")
            self.server.set_succeeded(self.result)
        else:
            rospy.loginfo("2024_climbing_server: failed")
            self.server.set_aborted(self.result)
        
if __name__ == '__main__':
    rospy.init_node('climbing_server_2024')
    server = ClimbAction(rospy.get_name())
    rospy.spin()