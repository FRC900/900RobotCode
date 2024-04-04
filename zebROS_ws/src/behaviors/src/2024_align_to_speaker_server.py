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
import behavior_actions.msg
from tf.transformations import euler_from_quaternion # may look like tf1 but is actually tf2
from frc_msgs.msg import MatchSpecificData
import angles
import numpy

class Aligner:
    # create messages that are used to publish feedback/result
    _feedback = behavior_actions.msg.AlignToSpeaker2024Feedback()
    _result = behavior_actions.msg.AlignToSpeaker2024Result()

    def __init__(self, name):   
        self._action_name = name
        self.tolerance = rospy.get_param("tolerance")
        self.velocity_tolerance = rospy.get_param("velocity_tolerance")
        self.min_samples = rospy.get_param("min_samples")
        self.color = 0
        self._as = actionlib.SimpleActionServer(self._action_name, behavior_actions.msg.AlignToSpeaker2024Action, execute_cb=self.aligner_callback, auto_start = False)
        self._as.start()
        self.current_yaw = 0
        self.angle_setpoint = 0
        self.current_orient_effort = 0
        self.valid_samples = 0
        self.offsetting = False
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.orientation_publish = rospy.Publisher("/teleop/orientation_command", std_msgs.msg.Float64, queue_size =1)
        self.imu_subscribe = rospy.Subscriber("/imu/zeroed_imu", sensor_msgs.msg.Imu, self.imu_callback, tcp_nodelay=True)

        self.team_subscribe = rospy.Subscriber("/frcrobot_rio/match_data", MatchSpecificData, self.data_callback)

        self.sub_effort = rospy.Subscriber("/teleop/orient_strafing/control_effort", std_msgs.msg.Float64, self.robot_orientation_effort_callback, tcp_nodelay=True)
        self.pub_cmd_vel = rospy.Publisher("/speaker_align/cmd_vel", geometry_msgs.msg.Twist, queue_size=1)

        self.pub_dist_and_ang_vel = rospy.Publisher("/speaker_align/dist_and_ang", behavior_actions.msg.AutoAlignSpeaker, queue_size=1) #distance and angle

        self._feedback.aligned = False
        
        self.feed_forward = True

    def imu_callback(self, imu_msg):
        q = imu_msg.orientation
        euler = euler_from_quaternion([q.x, q.y, q.z, q.w]) 
        yaw = euler[2]

        offset_point = tf2_geometry_msgs.PointStamped()
        offset_point.header.frame_id = 'bluespeaker' if self.color == 1 else 'redspeaker'
        offset_point.header.stamp = rospy.get_rostime()
        if self.offsetting:
            offset_point.point.y = 2
        else:
            offset_point.point.y = 0

        try:
            trans = self.tfBuffer.lookup_transform('base_link', offset_point.header.frame_id, rospy.Time())
            destination = tf2_geometry_msgs.do_transform_point(offset_point, trans)

            dist_ang_msg = behavior_actions.msg.AutoAlignSpeaker()
            dist_ang_msg.distance = math.hypot(destination.point.x, destination.point.y)
            dist_ang_msg.angle    = math.atan2(destination.point.y, destination.point.x)
            self.pub_dist_and_ang_vel.publish(dist_ang_msg)

            self.angle_setpoint = math.pi + yaw + math.atan2(destination.point.y, destination.point.x)
            self._feedback.error = abs(angles.shortest_angular_distance(self.angle_setpoint, yaw))
            if self._feedback.error < self.tolerance and abs(self.current_orient_effort) < self.velocity_tolerance:
                self.valid_samples += 1
            else:
                self.valid_samples = 0
            #rospy.loginfo(f"err {self._feedback.error} tolerance {self.tolerance}")
        except Exception as e:
            rospy.logwarn_throttle(1, f"align_to_speaker: can't publish distance {e}")
        
    def data_callback(self, data_msg):
        self.color = data_msg.allianceColor
        
    def robot_orientation_effort_callback(self, msg):
        self.current_orient_effort = msg.data
    
    def aligner_callback(self, goal: behavior_actions.msg.AlignToSpeaker2024Goal):
        self.feed_forward = True
        success = True
        rospy.loginfo(f"Auto Aligning Actionlib called with goal {goal}")
        rate = rospy.Rate(60.0)

        self.valid_samples = 0

        self._feedback.aligned = False
        self._as.publish_feedback(self._feedback)

        self.offsetting = goal.offsetting

        while not rospy.is_shutdown():
            # check that preempt has not been requested by the client
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                success = False
                break

            if self.valid_samples >= self.min_samples:
                self._feedback.aligned = True
                rospy.loginfo_throttle(0.5, "2024_align_to_speaker: aligned")
                self.feed_forward = False
                if not goal.align_forever:
                    # moved down here because we don't want to exit if align forever is true
                    success = True
                    cmd_vel_msg = geometry_msgs.msg.Twist()
                    cmd_vel_msg.angular.x = 0
                    cmd_vel_msg.angular.y = 0
                    cmd_vel_msg.angular.z = 0.0
                    cmd_vel_msg.linear.x = 0
                    cmd_vel_msg.linear.y = 0
                    cmd_vel_msg.linear.z = 0
                    self.pub_cmd_vel.publish(cmd_vel_msg)
                    self._as.publish_feedback(self._feedback)
                    break

            # If we haven't succeeded yet or we want to keep aligning forever, we need to
            # publish a cmd_vel message to make and/or keep the robot aligned

            # self.angle_setpoint is calulated in imu_callback and is the angle to the speaker
            # Publish it here since the imu_callback runs regardless if the action is active or not
            self.orientation_publish.publish(self.angle_setpoint)

            cmd_vel_msg = geometry_msgs.msg.Twist()
            cmd_vel_msg.angular.x = 0
            cmd_vel_msg.angular.y = 0
            cmd_vel_msg.angular.z = self.current_orient_effort
            if self.current_orient_effort > self.velocity_tolerance:
                cmd_vel_msg.angular.z += 0.5 * numpy.sign(self.current_orient_effort) * int(self.feed_forward)
            cmd_vel_msg.linear.x = 0
            cmd_vel_msg.linear.y = 0
            cmd_vel_msg.linear.z = 0
            self.pub_cmd_vel.publish(cmd_vel_msg)

            #rospy.loginfo_throttle(0.5, f"Align to speaker {abs(angles.shortest_angular_distance(self.msg.data, self.current_yaw))}")
            
            self._as.publish_feedback(self._feedback)
            rate.sleep()

        if success:
            self._result.success = True
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)
        
if __name__ == '__main__':
    rospy.init_node('aligner')
    server = Aligner(rospy.get_name())
    rospy.spin()