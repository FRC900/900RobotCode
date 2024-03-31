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
from ddynamic_reconfigure_python.ddynamic_reconfigure import DDynamicReconfigure
import behavior_actions.msg
from tf.transformations import euler_from_quaternion # may look like tf1 but is actually tf2
from frc_msgs.msg import MatchSpecificData
import std_srvs.srv
import angles
import numpy

class Aligner:
    # create messages that are used to publish feedback/result
    _feedback = behavior_actions.msg.AlignToSpeaker2024Feedback()
    _result = behavior_actions.msg.AlignToSpeaker2024Result()

    def __init__(self, name):   
        self.msg = None
        self.valid_samples = 0
        self._action_name = name
        self.tolerance = rospy.get_param("tolerance")
        self.velocity_tolerance = rospy.get_param("velocity_tolerance")
        self.min_samples = rospy.get_param("min_samples")
        self.dynamic_move_time = rospy.get_param("dynamic_move_time")
        self.offset_angle_radians = rospy.get_param("offset_angle_radians")

        self.color = 0
        self._as = actionlib.SimpleActionServer(self._action_name, behavior_actions.msg.AlignToSpeaker2024Action, execute_cb=self.aligner_callback, auto_start = False)
        self._as.start()
        self.current_yaw = 0
        self.current_orient_effort = 0
        self.x_field_relative_vel_imu = 0
        self.y_field_relative_vel_imu = 0

        self.x_field_relative_vel_align = 0
        self.y_field_relative_vel_align = 0

        self.angle_dist_x = 0
        self.angle_dist_y = 0

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.object_publish = rospy.Publisher("/teleop/orientation_command", std_msgs.msg.Float64, queue_size =1)
        self.object_subscribe = rospy.Subscriber("/imu/zeroed_imu", sensor_msgs.msg.Imu, self.imu_callback, tcp_nodelay=True)

        self.team_subscribe = rospy.Subscriber("/frcrobot_rio/match_data", MatchSpecificData, self.data_callback)

        self.sub_effort = rospy.Subscriber("/teleop/orient_strafing/control_effort", std_msgs.msg.Float64, self.robot_orientation_effort_callback, tcp_nodelay=True)
        self.pub_cmd_vel = rospy.Publisher("/speaker_align/cmd_vel", geometry_msgs.msg.Twist, queue_size=1)

        #for shooting while moving
        self.current_robot_cmd_vel = 0
        self.robot_cmd_vel = rospy.Subscriber("/frcrobot_jetson/swerve_drive_controller/cmd_vel_out", geometry_msgs.msg.TwistStamped, self.robot_cmd_vel_callback )    #is twist stamped the correct type of output???
        self.pub_dist_and_ang_vel = rospy.Publisher("/speaker_align/dist_and_ang", behavior_actions.msg.AutoAlignSpeaker, queue_size=1) #distance and angle
        self.feedback_error_value

        self._feedback.aligned = False
        
        self.feed_forward = True
        self.move_time_reconfigured = False

        ddynrec = DDynamicReconfigure("align_to_speaker_sever_dyn_rec")
        ddynrec.add_variable("tolerance", "float/double variable", rospy.get_param("tolerance"), 0.0, 3.0)
        ddynrec.add_variable("dynamic_move_time", "float/double variable", rospy.get_param("dynamic_move_time"), 0.0, 3.0)
        ddynrec.add_variable("offset_angle_radians", "float/double variable", rospy.get_param("offset_angle_radians"), 0.0, 3.14)
        self.dynamic_move_time = rospy.get_param("dynamic_move_time")

        self.move_time_reconfigured = False

        ddynrec = DDynamicReconfigure("align_to_speaker_sever_dyn_rec")
        ddynrec.add_variable("tolerance", "float/double variable", rospy.get_param("tolerance"), 0.0, 3.0)
        ddynrec.add_variable("dynamic_move_time", "float/double variable", rospy.get_param("dynamic_move_time"), 0.0, 3.0)
        ddynrec.start(self.dyn_rec_callback)

    def dyn_rec_callback(self, config, level):
        rospy.loginfo("Received reconf call: " + str(config))
        self.tolerance = config["tolerance"]
        self.dynamic_move_time = config["dynamic_move_time"]

        self.move_time_reconfigured = True
        return config

    def imu_callback(self, imu_msg):
        q = imu_msg.orientation
        euler = euler_from_quaternion([q.x, q.y, q.z, q.w]) 
        yaw = euler[2]
        self.current_yaw = yaw

        offset_point = tf2_geometry_msgs.PointStamped()
        offset_point.header.frame_id = 'bluespeaker' if self.color == 1 else 'redspeaker'
        offset_point.header.stamp = rospy.get_rostime()
        offset_point.point.y = 0

        try:
            trans = self.tfBuffer.lookup_transform('base_link', offset_point.header.frame_id, rospy.Time())
            destination = tf2_geometry_msgs.do_transform_point(offset_point, trans)

            self.msg = std_msgs.msg.Float64()
            dist_ang_msg = behavior_actions.msg.AutoAlignSpeaker()

            #rotate angle should just be self.current_yaw, though the sign may or may not be negative due to the amount of negatives that we have throughout code
            self.x_field_relative_vel_imu = self.current_robot_cmd_vel.linear.x * math.cos(-(self.current_yaw)) - self.current_robot_cmd_vel.linear.y * math.sin(-(self.current_yaw))
            self.y_field_relative_vel_imu = self.current_robot_cmd_vel.linear.x * math.sin(-(self.current_yaw)) + self.current_robot_cmd_vel.linear.y * math.cos(-(self.current_yaw))
        

            p_distance = math.sqrt(destination.point.x ** 2 + destination.point.y ** 2)
            v_distance = math.hypot(self.x_field_relative_vel_imu, self.y_field_relative_vel_imu)(self.dynamic_move_time)

            angle_dist_x = (self.x_field_relative_vel_imu)(self.dynamic_move_time) + destination.point.x
            angle_dist_y = (self.y_field_relative_vel_imu)(self.dynamic_move_time) + destination.point.y
            dist_ang_msg.distance =  v_distance + p_distance
            self.msg.data = math.pi + self.current_yaw + math.atan2(destination.point.y, destination.point.x)
            offset_angle = self.y_field_relative_vel_align * self.offset_angle_radians

            dist_ang_msg.angle = math.pi + self.current_yaw + math.atan2(destination.point.y, destination.point.x)
            #accounting for moving cases? 
            #dist_ang_msg.angle = math.atan2(angle_dist_y, angle_dist_x)

            dist_ang_msg.dynamic_move_while_shoot_time = rospy.get_time()

            #for this to be accurate, in .5 seconds, we have to moving at the same velocity, in addition to this, we shoot, in .5 seconds

            #here we use the kiematic equation to find hte distance that we are going to be at in t time, where t is predefined 
            #(v_o)(t) + dist_ang_msg.distance
            #(cmd_vel)(time_value) + dist_ang_msg.distance
            #
            self.pub_dist_and_ang_vel.publish(dist_ang_msg)
        except Exception as e:
            rospy.logwarn_throttle(1, f"align_to_speaker: can't publish distance {e}")

        if self.msg is not None:
            self._feedback.error = abs(angles.shortest_angular_distance(self.msg.data, self.current_yaw))
            self.feedback_error_value = self._feedback.error

            #need to publish this feedback error values

        #rospy.loginfo(f"errr {self._feedback.error} tolerance {self.tolerance}")
        if self._feedback.error < self.tolerance and abs(self.current_orient_effort) < self.velocity_tolerance:
            self.valid_samples += 1
        else:
            self.valid_samples = 0
        
    def data_callback(self, data_msg):
        self.color = data_msg.allianceColor
        
    def robot_orientation_effort_callback(self, msg):
        self.current_orient_effort = msg.data

    def robot_cmd_vel_callback(self, msg):
        self.current_robot_cmd_vel = msg.twist

    def aligner_callback(self, goal: behavior_actions.msg.AlignToSpeaker2024Goal):
        self.feed_forward = True
        success = True
        rospy.loginfo(f"Auto Aligning Actionlib called with goal {goal}")
        rate = rospy.Rate(60.0)

        self.valid_samples = 0

        while not rospy.is_shutdown():
            # check that preempt has not been requested by the client
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                success = False
                break
            offset_point = tf2_geometry_msgs.PointStamped()
            offset_point.header.frame_id = 'bluespeaker' if self.color == 1 else 'redspeaker'
            offset_point.header.stamp = rospy.get_rostime()

            if goal.offsetting == True:
                offset_point.point.y = 2
            else:
                offset_point.point.y = 0

            try:
                trans = self.tfBuffer.lookup_transform('base_link', offset_point.header.frame_id, rospy.Time())
                destination = tf2_geometry_msgs.do_transform_point(offset_point, trans)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                rospy.logwarn(e)
                rate.sleep()
                continue

            self.msg = std_msgs.msg.Float64()
            dist_ang_msg = behavior_actions.msg.AutoAlignSpeaker()

            self.x_field_relative_vel_align = self.current_robot_cmd_vel.linear.x * math.cos(-(self.current_yaw)) - self.current_robot_cmd_vel.linear.y * math.sin(-(self.current_yaw))
            self.y_field_relative_vel_align = self.current_robot_cmd_vel.linear.x * math.sin(-(self.current_yaw)) + self.current_robot_cmd_vel.linear.y * math.cos(-(self.current_yaw))

            self.angle_dist_x = (self.x_field_relative_vel_align * self.dynamic_move_time) + destination.point.x
            self.angle_dist_y = (self.y_field_relative_vel_align * self.dynamic_move_time) + destination.point.y
            self.msg.data = math.pi + self.current_yaw + math.atan2(destination.point.y, destination.point.x)
            #accounting for moving cases? 
            #dist_ang_msg.angle = math.atan2(y_field_relative_vel, x_field_relative_vel)
            #dist_ang_msg.angle = math.atan2(self.angle_dist_y, self.angle_dist_x)

            p_distance = math.sqrt(destination.point.x ** 2 + destination.point.y ** 2)
            v_distance = math.hypot(self.x_field_relative_vel_align, self.y_field_relative_vel_align) * (self.dynamic_move_time)
            dist_ang_msg.distance =  v_distance + p_distance
            dist_ang_msg.x_vel = self.x_field_relative_vel_align
            dist_ang_msg.y_vel = self.y_field_relative_vel_align

            #self.angle_dist_x = (self.x_field_relative_vel_align * self.dynamic_move_time) + destination.point.x
            #self.angle_dist_y = (self.y_field_relative_vel_align * self.dynamic_move_time) + destination.point.y
            #so, take the self.x_field_relative_vel_align value and multiply that by some offset angle in radians?
            #we're probably going to be moving 1 m/s or 2 m/s so this radian value could be fairly small?
            #make it dynamic
            #so increase the angle by which we adjust by mulitplying (self.x_field_relative_vel_align) * self.offset_angle_radians
            #which is the additional angle that we're going to add onto the angle that we already have on the align to angle callback?
            #so just add given value to hte angle value?
            #calculate readjustmnet angle using 15 degrees as the constant offset angle

            offset_angle = self.y_field_relative_vel_align * self.offset_angle_radians
            dist_ang_msg.angle = math.pi + self.current_yaw + math.atan2(destination.point.y, destination.point.x)
            #actually gets the angle that is being sent to really align to the speaker
            #removed the offset angle for now


            #accounting for moving cases? 
            #dist_ang_msg.angle = math.atan2(y_field_relative_vel, x_field_relative_vel)
            #dist_ang_msg.angle = math.atan2(self.angle_dist_y, self.angle_dist_x)

            p_distance = math.sqrt(destination.point.x ** 2 + destination.point.y ** 2)
            v_distance = math.hypot(self.x_field_relative_vel_align, self.y_field_relative_vel_align) * (self.dynamic_move_time)
            dist_ang_msg.distance =  v_distance + p_distance
            dist_ang_msg.x_vel = self.x_field_relative_vel_align
            dist_ang_msg.y_vel = self.y_field_relative_vel_align
            dist_ang_msg.offset_angle = offset_angle
            dist_ang_msg.offset_angle_degrees = math.degrees(offset_angle)

            dist_ang_msg.destination_y = destination.point.y
            dist_ang_msg.destination_x = destination.point.x

            dist_ang_msg.degree_angle = math.degrees(dist_ang_msg.angle)
            self._feedback.aligned = False
            #all of these variables here are being set so that you can pull said values from the auto align speaker action




            if goal.mode == 0:#having this on lets the moving while shooting align actually work
                self.object_publish.publish(self.msg) #this line actually sends the robot to really align to the values are specified, let us test this and make sure just to be safe

            elif goal.mode == 1:
                self.object_publish.publish(self.msg)
            self.pub_dist_and_ang_vel.publish(dist_ang_msg) #always publishes the angle that the robot is supposed to align to, the object_publish objct line actually makes the robot align
            
            cmd_vel_msg = geometry_msgs.msg.Twist()
            cmd_vel_msg.angular.x = 0
            cmd_vel_msg.angular.y = 0
            cmd_vel_msg.angular.z = self.current_orient_effort
            if abs(self._feedback.error) > 0.1: 
                cmd_vel_msg.angular.z += 1.0 * numpy.sign(self.current_orient_effort) * int(self.feed_forward)
            cmd_vel_msg.linear.x = 0
            cmd_vel_msg.linear.y = 0
            cmd_vel_msg.linear.z = 0
            if goal.mode == 1:
                self.pub_cmd_vel.publish(cmd_vel_msg)

            rospy.loginfo_throttle(0.5, f"Align to speaker {abs(angles.shortest_angular_distance(self.msg.data, self.current_yaw))}")

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