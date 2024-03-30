#! /usr/bin/env python3

import actionlib
import rospy
import math
import tf2_ros
import tf2_geometry_msgs
import geometry_msgs.msg
import std_msgs.msg
import sensor_msgs.msg
import math
import time
from behavior_actions.msg import DriveAndScore2024Action, DriveAndScore2024Goal, DriveAndScore2024Feedback, DriveAndScore2024Result 
import behavior_actions.msg
from tf.transformations import euler_from_quaternion # may look like tf1 but is actually tf2
from frc_msgs.msg import MatchSpecificData
import std_srvs.srv
import angles
from controllers_2024_msgs.srv import ShooterPivotSrv, ShooterPivotSrvRequest
import numpy
import geometry_msgs.msg

# While the snap to angle button is held, spin up the shooter for amp and override driver
# Once amp shot button is pressed, call shooting server with goal=AMP. make shooting server drive backwards for amp.

# Used for anytime we need to drive to a position and then score, (amp/trap) as opposed to shooting where we just need to align (2024_align_to_speaker_server.py and other)

class DriveAndScore:
    # create messages that are used to publish feedback/result
    _feedback = DriveAndScore2024Feedback()
    _result = DriveAndScore2024Result()

    def __init__(self, name):   
        self.valid_samples = 0
        self._action_name = name
        self.preempted = False
        # get us to right spot
        self.align_client = actionlib.SimpleActionClient('/align_to_trap/align_to_trap_2024', behavior_actions.msg.AlignToTrap2024Action)
        rospy.loginfo("2024_intaking_server: waiting for shooter pivot server")
        self.align_client.wait_for_server()
        # shoot trap
        self.shooting_client = actionlib.SimpleActionClient('/shooting/shooting_server_2024', behavior_actions.msg.Shooting2024Action)
        
        self.leafblower_pub = rospy.Publisher("/frcrobot_rio/leafblower_controller/command", std_msgs.msg.Float64, queue_size=1)
        self.arm_pivot_client = rospy.ServiceProxy("/frcrobot_jetson/arm_controller/shooter_pivot_service", ShooterPivotSrv)

        self.trap_blower_rest_position: float = rospy.get_param("trap_blower_rest_position")
        self.trap_blower_position: float = rospy.get_param("trap_blower_position")
        self.trap_blower_speed: int = rospy.get_param("trap_blower_speed")

        rospy.loginfo("2024_intaking_server: waiting for shooting server")
        self.shooting_client.wait_for_server()
        self.pub_cmd_vel = rospy.Publisher("/auto_note_align/cmd_vel", geometry_msgs.msg.Twist, queue_size=1)        
        self.align_done = False
        
        self.shooting_done = False 
        self._as = actionlib.SimpleActionServer(self._action_name, DriveAndScore2024Action, execute_cb=self.score_cb, auto_start = False)
        self._as.start()
    
    def align_done_cb(self, status, result):
        rospy.loginfo("2024 drive and score, align server done")
        self.align_done = True
    
    def shooting_done_cb(self, status, result):
        self.shooting_done = True
    
    def trap_shooting_done_cb(self, status, result):
        print(status)
        print(result)
        if self.preempted:
            rospy.logwarn("Preempted, not running cleanup in drive and score")
            return
        time.sleep(0.75)
        r = rospy.Rate(60.0)
        cmd_vel_msg = geometry_msgs.msg.Twist()
        start = rospy.Time.now()
        pivot = ShooterPivotSrvRequest()
        pivot.angle = self.trap_blower_rest_position
        if not self.arm_pivot_client.call(pivot):
            rospy.logerr("2024 drive and score - failed to send arm back, this is very bad")
            self._result.success = False
            self._as.set_succeeded(self._result)
            return
        while (rospy.Time.now() - start < rospy.Duration(0.75)):
            rospy.loginfo("Driving back!")
            cmd_vel_msg.angular.x = 0
            cmd_vel_msg.angular.y = 0
            cmd_vel_msg.angular.z = 0
            cmd_vel_msg.linear.x = -2.0
            cmd_vel_msg.linear.y = 0
            cmd_vel_msg.linear.z = 0
            self.pub_cmd_vel.publish(cmd_vel_msg)
            r.sleep()
        rospy.loginfo("2024 drive and score, trap shooting done, shutting down leafblower and pivoting arm")
        msg = std_msgs.msg.Float64()
        msg.data = 0.0 
        self.leafblower_pub.publish(msg)


    def score_cb(self, goal: DriveAndScore2024Goal):
        self.preempted = False
        self.feed_forward = True
        success = True
        self.align_done = False
        self.shooting_done = False
        have_shot = False 
        rospy.loginfo(f"Drive and score 2024 - called with goal {goal}")
        r = rospy.Rate(60.0)
        align_goal = behavior_actions.msg.AlignToTrap2024Goal()

        if goal.destination == goal.AMP:
            rospy.loginfo(f"Drive and score 2024 - going amp")
            align_goal.destination = align_goal.AMP
            self.align_client.send_goal(align_goal, done_cb=self.align_done_cb)
            shooting_goal = behavior_actions.msg.Shooting2024Goal()
            shooting_goal.mode = shooting_goal.AMP
            shooting_goal.leave_spinning = True
            shooting_goal.setup_only = True
            self.shooting_client.send_goal(shooting_goal)

        elif goal.destination == goal.TRAP:
            rospy.loginfo(f"Drive and score 2024 - going trap")
            align_goal.destination = align_goal.TRAP
            self.align_client.send_goal(align_goal, done_cb=self.align_done_cb)
            msg = std_msgs.msg.Float64()
            rospy.loginfo(f"Blowing up leafblower to: {self.trap_blower_speed}")
            msg.data = self.trap_blower_speed
            self.leafblower_pub.publish(msg)
            pivot = ShooterPivotSrvRequest()
            pivot.angle = self.trap_blower_position
            if not self.arm_pivot_client.call(pivot):
                rospy.logerr("2024 drive and score - failed to call arm pivot client")
                self._result.success = False
                self._as.set_succeeded(self._result)
                return
            
        # for telling if we hvae 
        while not rospy.is_shutdown():
            # check that preempt has not been requested by the client
            if self._as.is_preempt_requested():
                self.preempted = True
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                self.align_client.cancel_goals_at_and_before_time(rospy.Time.now())
                self.shooting_client.cancel_goals_at_and_before_time(rospy.Time.now())
                rospy.loginfo("Shutting down leafblower")
                msg = std_msgs.msg.Float64()
                msg.data = 0.0 
                self.leafblower_pub.publish(msg)
                success = False
                pivot = ShooterPivotSrvRequest()
                pivot.angle = self.trap_blower_rest_position
                if not self.arm_pivot_client.call(pivot):
                    rospy.logerr("2024 drive and score - failed to send arm back, this is very bad")
                    self._result.success = False
                    self._as.set_succeeded(self._result)
                    return
                cmd_vel_msg = geometry_msgs.msg.Twist()
                start = rospy.Time.now()
                while (rospy.Time.now() - start < rospy.Duration(0.5)):
                    rospy.loginfo("Driving back in preempt!")
                    cmd_vel_msg.angular.x = 0
                    cmd_vel_msg.angular.y = 0
                    cmd_vel_msg.angular.z = 0
                    cmd_vel_msg.linear.x = -2.0
                    cmd_vel_msg.linear.y = 0
                    cmd_vel_msg.linear.z = 0
                    self.pub_cmd_vel.publish(cmd_vel_msg)
                    r.sleep()
                return
                
            rospy.loginfo_throttle(0.1, f"2024_align_and_SCORE: waiting on {'aligning' if not self.align_done else ''} {'shooting' if not self.shooting_done  and goal.destination == goal.TRAP else ''}")

            # should be good to outake
            if goal.destination == goal.AMP and self.align_done and not have_shot:
                rospy.loginfo("Drive and score 2024 - align and amp done")
                cmd_vel_msg = geometry_msgs.msg.Twist()
                start = rospy.Time.now()

                while (rospy.Time.now() - start < rospy.Duration(0.5)):
                    cmd_vel_msg.angular.x = 0
                    cmd_vel_msg.angular.y = 0
                    cmd_vel_msg.angular.z = 0
                    cmd_vel_msg.linear.x = -1.0
                    cmd_vel_msg.linear.y = 0
                    cmd_vel_msg.linear.z = 0
                    self.pub_cmd_vel.publish(cmd_vel_msg)
                    r.sleep()
                shooting_goal = behavior_actions.msg.Shooting2024Goal()
                shooting_goal.mode = shooting_goal.AMP
                shooting_goal.leave_spinning = False
                self.shooting_client.send_goal(shooting_goal, done_cb=self.shooting_done_cb)
                have_shot = True

            if goal.destination == goal.AMP and self.align_done and self.shooting_done:
                rospy.loginfo("DRIVE AND SCORE, shooting and align done")
                # drive back
                cmd_vel_msg = geometry_msgs.msg.Twist()
                start = rospy.Time.now()

                while (rospy.Time.now() - start < rospy.Duration(0.25)):
                    cmd_vel_msg.angular.x = 0
                    cmd_vel_msg.angular.y = 0
                    cmd_vel_msg.angular.z = 0
                    cmd_vel_msg.linear.x = 2.0
                    cmd_vel_msg.linear.y = 0
                    cmd_vel_msg.linear.z = 0
                    self.pub_cmd_vel.publish(cmd_vel_msg)
                    r.sleep()

                self._result.success = True
                self._as.set_succeeded(self._result)
                return 

            if goal.destination == goal.TRAP and self.align_done:
                rospy.loginfo("2024 drive and score - trap aligned, shooting")
                # need to shoot
                shooting_goal = behavior_actions.msg.Shooting2024Goal()
                shooting_goal.mode = shooting_goal.TRAP
                shooting_goal.leave_spinning = False
                shooting_goal.setup_only = False
                self.shooting_client.send_goal(shooting_goal, done_cb=self.trap_shooting_done_cb)
            
                self._result.success = True
                self._as.set_succeeded(self._result)
                return

            self._as.publish_feedback(self._feedback)
            r.sleep()

        if success:
            r.sleep()
            self._result.success = True
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)
        
if __name__ == '__main__':
    rospy.init_node('drive_and_score')
    server = DriveAndScore(rospy.get_name())
    rospy.spin()