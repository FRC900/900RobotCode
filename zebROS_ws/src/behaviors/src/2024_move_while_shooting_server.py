#!/usr/bin/env python3

import rospy
import actionlib
import time
import math
import std_msgs.msg
import behavior_actions.msg
import numpy

from behavior_actions.msg import Shooting2024Goal, Shooting2024Feedback, Shooting2024Result, Shooting2024Action
from behavior_actions.msg import MoveWhileShooting2024Goal, MoveWhileShooting2024Feedback, MoveWhileShooting2024Result, MoveWhileShooting2024Action
from behavior_actions.msg import AlignToSpeaker2024Goal, AlignToSpeaker2024Feedback, AlignToSpeaker2024Result, AlignToSpeaker2024Action
from behavior_actions.msg import AutoAlignSpeaker
from std_msgs.msg import Float64
from interpolating_map import InterpolatingMap
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped


class ShootingServer(object):
    def __init__(self, name):
        self.action_name = name
        self.result = MoveWhileShooting2024Result()
        self.feedback = MoveWhileShooting2024Feedback()

        self.dynamic_move_time = rospy.get_param("dynamic_move_time")
        self.cmd_vel_pub = rospy.Publisher("/auto_note_align/cmd_vel", Twist, queue_size=1, tcp_nodelay=True)
        #self.cmd_vel_pub_ = rospy.Publisher("/auto_note_align/cmd_vel", geometry_msgs.msg.Twist, queue_size=1)


        self.negative = None

        self.object_publish = rospy.Publisher("/teleop/orientation_command", std_msgs.msg.Float64, queue_size =1)

        self.angle_holder = std_msgs.msg.Float64()

        self.cmd_vel_sub = rospy.Subscriber("/frcrobot_jetson/swerve_drive_controller/cmd_vel_out", TwistStamped, self.cmd_vel_sub_magnitude_convert_callback, tcp_nodelay=True, queue_size=1)
        self.scaled_x_val = 0.0
        self.scaled_y_val = 0.0
        #creating cmd_vel_sub, to read in the values of the velocity values whem moving in sim, then taking callback and scaling magnitude to 1 m/s

        self.angle_puller = rospy.Subscriber("/speaker_align/dist_and_ang", AutoAlignSpeaker, self.dist_and_ang_cb, tcp_nodelay=True ,queue_size=1)
        self.angle_cb = 0.0
        self.distance_cb = 0.0
        self.feedback_error_value = 0.0
        #creating angle_puller to subscribe nad read twist values, this should align the robot accorindlgy

        #        self.pub_cmd_vel = rospy.Publisher("/speaker_align/cmd_vel", geometry_msgs.msg.Twist, queue_size=1)
        self.sub_cmd_vel = rospy.Subscriber("/speaker_align/cmd_vel", Twist, self.correction_ang_cb, tcp_nodelay=True, queue_size=1)
        self.current_orient_effort_cb = 0.0

        self.shooting_client = actionlib.SimpleActionClient('/shooting/shooting_server_2024', Shooting2024Action)
        rospy.loginfo("Waiting for shooting server")
        self.shooting_client.wait_for_server()

        self.align_to_speaker_client = actionlib.SimpleActionClient('/align_to_speaker/align_to_speaker_2024', AlignToSpeaker2024Action)
        rospy.loginfo("Waiting for AlignToSpeaker")
        self.align_to_speaker_client.wait_for_server()

        self.server = actionlib.SimpleActionServer(self.action_name, MoveWhileShooting2024Action, execute_cb=self.execute_cb, auto_start = False)
        self.server.start()
        rospy.loginfo("2024_move_while_shooting_server: initialized")

    def align_to_speaker_feedback_cb(self, feedback: AlignToSpeaker2024Feedback):
        self.align_to_speaker_done = feedback.aligned

    def shooting_feedback_cb(self, feedback: Shooting2024Feedback):
        self.shooting_done = (feedback.current_stage == feedback.SHOOTING)
        rospy.logwarn("2024 Align and shoot: Shooting done!")

    def correction_ang_cb(self, msg):
        rospy.loginfo("2024_move_while_shooting_server.py got correctoin_ang_cb")
        #angular.z
        self.current_orient_effort_cb = msg.angular.z

    def dist_and_ang_cb(self, msg):
        self.angle_cb = msg.angle
        self.distane_cb = msg.distance

        self.angle_holder = self.angle_cb #might have to use this one?
        self.feedback_error_value = msg.feedback_error_value
        rospy.loginfo("self.angle_twist_z has the new angle")
        
    def cmd_vel_sub_magnitude_convert_callback(self, msg: TwistStamped):
        #just find unit vector
        #is it even msg.linear.x?
        #the subscriber callback takes in objects of geometry_msgs.Twiststamped
        x_val = msg.twist.linear.x
        y_val = msg.twist.linear.y
        if (x_val == 0) or (y_val == 0):
            self.scaled_x_val = 0
            self.scaled_y_val = 0
        if ((x_val > 0) or (y_val > 0)):
            vector_magnitude = math.hypot(x_val, y_val)
            if vector_magnitude > 0:
                self.scaled_x_val = x_val / vector_magnitude
                #perhaps do transform on these things for field relative? would prevent drift
                rospy.loginfo("2024_shooting_server, convert callback")
                self.scaled_y_val = y_val / vector_magnitude
                rospy.loginfo("2024_shooting_server, convert callback")


    def execute_cb(self, goal: MoveWhileShooting2024Goal):
        rospy.loginfo("move while shooting server 2024: has received execute cb")
        self.result.success = False
        cmd_vel_msg_move_and_shoot = Twist()
        shooting_goal = Shooting2024Goal()
        align_to_speaker_goal = AlignToSpeaker2024Goal()

        shooting_goal.distance = self.distance_cb
        shooting_goal.mode = shooting_goal.SPEAKER
        shooting_goal.setup_only = False
        shooting_goal.leave_spinning = False
        shooting_goal.request_time = rospy.get_time()
        shooting_done = False
        def shooting_done_cb(state, result):
            nonlocal shooting_done
            shooting_done = True
        self.shooting_client.send_goal(shooting_goal, done_cb=shooting_done_cb)
        rospy.loginfo("got here send shooting client cb")

        align_to_speaker_goal.align_forever = True
        align_to_speaker_goal.mode = 0
        align_to_speaker_goal.offsetting = False
        rospy.loginfo("got here, got status for the align to speaker goal")
        rospy.loginfo("2024 move while shooting server requesting align angle values")
        self.align_to_speaker_client.send_goal(align_to_speaker_goal, feedback_cb=self.align_to_speaker_feedback_cb)
        rospy.loginfo("align to speaker goal sent")
        #while conditoin with preempt and what not
        r = rospy.Rate(60.0)
        request_time = rospy.get_time()
        current_time = rospy.get_time()





        while ((current_time - request_time) < self.dynamic_move_time):
        #while True:
            rospy.loginfo("move while shooting server 2024: is in execute cb while loop")
            current_time = rospy.get_time()
            #while not enough time has passed, go through this entire loop

            if self.server.is_preempt_requested():
                #if preempted, cancel the goals that we send and make sure the stuff that we are doing is preempted
                rospy.loginfo("move while shooting server 2024: preempted")
                #self.shooting_client.cancel_goals_at_and_before_time(rospy.Time.now())
                self.shooting_client.cancel_goals_at_and_before_time(rospy.Time.now())
                self.align_to_speaker_client.cancel_goals_at_and_before_time(rospy.Time.now())
                self.server.set_preempted()
                cmd_vel_msg_move_and_shoot.linear.x = 0
                cmd_vel_msg_move_and_shoot.linear.y = 0
                cmd_vel_msg_move_and_shoot.angular.z = 0
                self.cmd_vel_pub.publish(cmd_vel_msg_move_and_shoot)
                return
            if goal.move_align == True:
                #if we do intend on moving, aligning and shooting then do the following
                rospy.loginfo("move while shooting server 2024: move_align is true, setting velocity and angular conditions")
                rospy.loginfo(f"this is the angle we are using to align: {self.angle_cb}")
                self.object_publish.publish(self.angle_holder) #main align command

                cmd_vel_msg_move_and_shoot.linear.x = self.scaled_x_val
                cmd_vel_msg_move_and_shoot.linear.y = self.scaled_y_val
                cmd_vel_msg_move_and_shoot.angular.z = self.current_orient_effort_cb #helps with microadjustments

                if abs(self.feedback_error_value) > 0.1: 
                    cmd_vel_msg_move_and_shoot.angular.z += 1.0 * numpy.sign(self.current_orient_effort) * int(self.feed_forward) #also helps with microadjustmnets

                self.cmd_vel_pub.publish(cmd_vel_msg_move_and_shoot)

            r.sleep() 

        self.result.success = True
        self.server.set_succeeded(self.result)
        return

if __name__ == '__main__':
    rospy.init_node('move_while_shooting_server_2024')
    server = ShootingServer(rospy.get_name())
    rospy.spin()