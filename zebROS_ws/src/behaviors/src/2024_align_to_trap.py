#! /usr/bin/env python3

import actionlib
import rospy
import math
import tf2_ros
import geometry_msgs.msg
import std_msgs.msg
import sensor_msgs.msg
import math
import behavior_actions.msg
from tf.transformations import euler_from_quaternion # may look like tf1 but is actually tf2
from tf import transformations as t
from frc_msgs.msg import MatchSpecificData
import std_srvs.srv

# Logic:
# - find closest trap alignment spot to current position (idea: just store distance from tag and use tf2 since we have all the tag transforms)
# - ideally, verify we don't run into anything
# - PID there

class Aligner:
    # create messages that are used to publish feedback/result
    _feedback = behavior_actions.msg.AlignToTrap2024Feedback()
    _result = behavior_actions.msg.AlignToTrap2024Result()

    RED_TAGS = ["red_trap_11", "red_trap_12", "red_trap_13"]
    BLUE_TAGS = ["blue_trap_14", "blue_trap_15", "blue_trap_16"]
    
    BLUE_AMP = "blue_amp_6"
    RED_AMP = "red_amp_5"

    BLUE_SUBWOOFER = "blue_subwoofer_7"
    RED_SUBWOOFER = "red_subwoofer_4"


    def __init__(self, name):   
        self._action_name = name
        self.x_tolerance = rospy.get_param("x_tolerance")
        self.y_tolerance = rospy.get_param("y_tolerance")
        self.angle_tolerance = rospy.get_param("angle_tolerance")

        self.x_offset = rospy.get_param("x_offset")
        self.y_offset = rospy.get_param("y_offset")

        self.x_amp_offset = rospy.get_param("x_offset_amp")
        self.y_amp_offset = rospy.get_param("y_offset_amp")

        self.x_subwoofer_offset = rospy.get_param("x_offset_subwoofer")
        
        self.color = 0
        self._as = actionlib.SimpleActionServer(self._action_name, behavior_actions.msg.AlignToTrap2024Action, execute_cb=self.aligner_callback, auto_start = False)
        self._as.start()
        
        self.current_yaw = 0
        self.current_orient_effort = 0
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.orientation_command_pub = rospy.Publisher("/teleop/orientation_command", std_msgs.msg.Float64, queue_size=1)
        self.object_subscribe = rospy.Subscriber("/imu/zeroed_imu", sensor_msgs.msg.Imu, self.imu_callback)

        self.x_state_pub = rospy.Publisher("x_position_pid/x_state_pub", std_msgs.msg.Float64, queue_size=1, tcp_nodelay=True)
        self.x_cmd_pub = rospy.Publisher("x_position_pid/x_cmd_pub", std_msgs.msg.Float64, queue_size=1, tcp_nodelay=True)
        self.x_enable_pub = rospy.Publisher("x_position_pid/x_enable_pub", std_msgs.msg.Bool, queue_size=1, tcp_nodelay=True)
        self.x_command = 0
        # self.x_command_sub = rospy.Publisher("x_position_pid/x_command", std_msgs.msg.Float64, self.x_command_callback, tcp_nodelay=True)

        self.y_state_pub = rospy.Publisher("y_position_pid/y_state_pub", std_msgs.msg.Float64, queue_size=1, tcp_nodelay=True)
        self.y_cmd_pub = rospy.Publisher("y_position_pid/y_cmd_pub", std_msgs.msg.Float64, queue_size=1, tcp_nodelay=True)
        self.y_enable_pub = rospy.Publisher("y_position_pid/y_enable_pub", std_msgs.msg.Bool, queue_size=1, tcp_nodelay=True)
        self.y_command = 0
        # self.y_command_sub = rospy.Publisher("y_position_pid/y_command", std_msgs.msg.Float64, self.y_command_callback, tcp_nodelay=True)

        self.team_subscribe = rospy.Subscriber("/frcrobot_rio/match_data", MatchSpecificData, self.match_data_callback)

        self.enable_pub = rospy.Publisher("align_to_trap_pid/pid_enable", std_msgs.msg.Bool, queue_size=1, tcp_nodelay=True)

        self.sub_effort = rospy.Subscriber("/teleop/orient_strafing/control_effort", std_msgs.msg.Float64, self.robot_orientation_effort_callback)

    def x_command_callback(self, msg: std_msgs.msg.Float64):
        self.x_command = msg.data

    def y_command_callback(self, msg: std_msgs.msg.Float64):
        self.y_command = msg.data

    def imu_callback(self, imu_msg):
        q = imu_msg.orientation
        euler = euler_from_quaternion([q.x, q.y, q.z, q.w]) 
        yaw = euler[2]
        self.current_yaw = yaw
    
    def match_data_callback(self, data_msg):
        self.color = data_msg.allianceColor
        
    def robot_orientation_effort_callback(self, msg):
        self.current_orient_effort = msg.data

    def aligner_callback(self, goal: behavior_actions.msg.AlignToTrap2024Goal):
        success = True
        rospy.loginfo(f"Auto Aligning Actionlib called with goal {goal}")
        rate = rospy.Rate(50.0)

        closest_frame = None
        closest_distance = float("inf")
        if goal.destination == goal.AMP:
            rospy.loginfo("2024_align_to_trap: Aligning to amp")
            closest_frame = self.RED_AMP if self.color == MatchSpecificData.ALLIANCE_COLOR_RED else self.BLUE_AMP
        elif goal.destination == goal.SUBWOOFER:
            rospy.loginfo("2024_align_to_trap: Aligning to subwoofer")
            closest_frame = self.RED_SUBWOOFER if self.color == MatchSpecificData.ALLIANCE_COLOR_RED else self.BLUE_SUBWOOFER
        else:
            rospy.loginfo("2024_align_to_trap: Aligning to trap")
            for frame in (self.RED_TAGS if self.color == MatchSpecificData.ALLIANCE_COLOR_RED else self.BLUE_TAGS):
                trans: geometry_msgs.msg.TransformStamped = self.tfBuffer.lookup_transform('base_link', frame, rospy.Time())
                distance = math.hypot(trans.transform.translation.x, trans.transform.translation.y)
                if distance < closest_distance:
                    closest_frame = frame
                    closest_distance = distance

        trans = self.tfBuffer.lookup_transform(closest_frame, "map", rospy.Time())
        if goal.destination == goal.TRAP:
            trans.transform.translation.x -= self.x_offset
            trans.transform.translation.y -= self.y_offset
        elif goal.destination == goal.AMP:
            trans.transform.translation.x -= self.x_amp_offset
            trans.transform.translation.y -= self.y_amp_offset
        elif goal.destination == goal.SUBWOOFER:
            trans.transform.translation.x -= self.x_subwoofer_offset

        transform = t.concatenate_matrices(t.translation_matrix([trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z]), t.quaternion_matrix([trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w]))
        inverted_transform = t.inverse_matrix(transform)

        inverted_translation = t.translation_from_matrix(inverted_transform)
        x_translation = inverted_translation[0]
        y_translation = inverted_translation[1]

        inverted_rotation = t.quaternion_from_matrix(inverted_transform)

        trap_yaw = t.euler_from_quaternion(inverted_rotation)[2]

        self.x_enable_pub.publish(std_msgs.msg.Bool(True))
        self.y_enable_pub.publish(std_msgs.msg.Bool(True))
        self.enable_pub.publish(std_msgs.msg.Bool(True))

        while not rospy.is_shutdown():
            # check that preempt has not been requested by the client
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                success = False
                break
            try:
                map_to_baselink: geometry_msgs.msg.TransformStamped = self.tfBuffer.lookup_transform("map", "base_link", rospy.Time())

            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                rospy.logwarn(e)
                rate.sleep()
                continue
                
            self.x_state_pub.publish(std_msgs.msg.Float64(map_to_baselink.transform.translation.x))
            self.x_cmd_pub.publish(std_msgs.msg.Float64(x_translation))

            self.y_state_pub.publish(std_msgs.msg.Float64(map_to_baselink.transform.translation.y))
            self.y_cmd_pub.publish(std_msgs.msg.Float64(y_translation))

            msg = std_msgs.msg.Float64()

            msg.data = trap_yaw

            self._feedback.angular_error = abs(trap_yaw - self.current_yaw) % (2 * math.pi)
            self._feedback.x_error = abs(x_translation - map_to_baselink.transform.translation.x)
            self._feedback.y_error = abs(y_translation - map_to_baselink.transform.translation.y)
            self._as.publish_feedback(self._feedback)

            self.orientation_command_pub.publish(msg)
            
            if self._feedback.angular_error < self.angle_tolerance and self._feedback.x_error < self.x_tolerance and self._feedback.y_error < self.y_tolerance:
                success = True
                break

            rate.sleep()
        if success:
            self._result.success = True
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)

        self.x_enable_pub.publish(std_msgs.msg.Bool(False))
        self.y_enable_pub.publish(std_msgs.msg.Bool(False))
        self.enable_pub.publish(std_msgs.msg.Bool(False))
        
if __name__ == '__main__':
    rospy.init_node('aligner')
    server = Aligner(rospy.get_name())
    rospy.spin()