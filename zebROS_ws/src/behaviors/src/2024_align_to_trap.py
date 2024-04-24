#! /usr/bin/env python3

import actionlib
import rospy
import math
import tf2_ros
import std_msgs.msg
import sensor_msgs.msg
import math
import behavior_actions.msg
from tf.transformations import euler_from_quaternion # may look like tf1 but is actually tf2
from frc_msgs.msg import MatchSpecificData
import norfair_ros.msg
import angles

# Logic:
# - find closest trap alignment spot to current position (idea: just store distance from tag and use tf2 since we have all the tag transforms)
# - ideally, verify we don't run into anything
# - PID there

# rosservice call /frcrobot_jetson/arm_controller/shooter_pivot_service "angle: 0.24434"
# rosservice call /frcrobot_rio/leafblower_controller/command "command: 1.0"

class Aligner:
    # create messages that are used to publish feedback/result
    _feedback = behavior_actions.msg.AlignToTrap2024Feedback()
    _result = behavior_actions.msg.AlignToTrap2024Result()

    RED_TAGS = [11, 12, 13]
    BLUE_TAGS = [14, 15, 16]
    RED_TRAP_YAW = list(map(math.radians, [-120, 0, 120]))
    BLUE_TRAP_YAW = list(map(math.radians, [-60, 60, 180]))
    
    BLUE_AMP = [6]
    RED_AMP = [5]
    AMP_YAW = [math.radians(-90)]

    BLUE_SUBWOOFER = [7]
    RED_SUBWOOFER = [4]
    BLUE_SUBWOOFER_YAW = [math.radians(0)]
    RED_SUBWOOFER_YAW = [math.radians(180)]

    def __init__(self, name):   
        self._action_name = name
        self.trap_x_tolerance = rospy.get_param("trap_x_tolerance")
        self.trap_y_tolerance = rospy.get_param("trap_y_tolerance")
        self.trap_angle_tolerance = rospy.get_param("trap_angle_tolerance")
        self.trap_min_x_vel = rospy.get_param("trap_min_x_vel")
        self.trap_min_y_vel = rospy.get_param("trap_min_y_vel")
        self.trap_fast_zone = rospy.get_param("trap_fast_zone")

        self.amp_x_tolerance = rospy.get_param("amp_x_tolerance")
        self.amp_y_tolerance = rospy.get_param("amp_y_tolerance")
        self.amp_angle_tolerance = rospy.get_param("amp_angle_tolerance")
        self.amp_min_x_vel = rospy.get_param("amp_min_x_vel")
        self.amp_min_y_vel = rospy.get_param("amp_min_y_vel")
        self.amp_fast_zone = rospy.get_param("amp_fast_zone")

        self.subwoofer_x_tolerance = rospy.get_param("subwoofer_x_tolerance")
        self.subwoofer_y_tolerance = rospy.get_param("subwoofer_y_tolerance")
        self.subwoofer_angle_tolerance = rospy.get_param("subwoofer_angle_tolerance")
        self.subwoofer_min_x_vel = rospy.get_param("subwoofer_min_x_vel")
        self.subwoofer_min_y_vel = rospy.get_param("subwoofer_min_y_vel")
        self.subwoofer_fast_zone = rospy.get_param("subwoofer_fast_zone")

        self.stage_1_trap_frame = rospy.get_param("stage_1_trap_frame")
        self.stage_2_trap_frame = rospy.get_param("stage_2_trap_frame")

        self.amp_frame = rospy.get_param("amp_frame")
        self.subwoofer_frame = rospy.get_param("subwoofer_frame")
        
        self.color = 0
        self._as = actionlib.SimpleActionServer(self._action_name, behavior_actions.msg.AlignToTrap2024Action, execute_cb=self.aligner_callback, auto_start = False)
        self._as.start()

        self.drive_to_object_client = actionlib.SimpleActionClient("/drive_to_object/drive_to_object", behavior_actions.msg.DriveToObjectAction)
        self.norfair_sub = rospy.Subscriber("/norfair/output", norfair_ros.msg.Detections, self.tracked_objects_callback)

        self.visible_objects = []
        
        self.current_yaw = 0
        self.current_orient_effort = 0
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.orientation_command_pub = rospy.Publisher("/teleop/orientation_command", std_msgs.msg.Float64, queue_size=1)
        self.object_subscribe = rospy.Subscriber("/imu/zeroed_imu", sensor_msgs.msg.Imu, self.imu_callback)

        self.team_subscribe = rospy.Subscriber("/frcrobot_rio/match_data", MatchSpecificData, self.match_data_callback)

    def match_data_callback(self, data_msg):
        self.color = data_msg.allianceColor

    def imu_callback(self, imu_msg):
        q = imu_msg.orientation
        euler = euler_from_quaternion([q.x, q.y, q.z, q.w]) 
        self.current_yaw = euler[2]

    def tracked_objects_callback(self, msg: norfair_ros.msg.Detections):
        self.visible_objects = [detection.label for detection in msg.detections]

    def aligner_callback(self, goal: behavior_actions.msg.AlignToTrap2024Goal):
        success = True
        rospy.loginfo(f"Auto Aligning Actionlib called with goal {goal}")
        rate = rospy.Rate(50.0)
        stage_2_trap = False
        closest_tag = None
        closest_distance = float("inf")
        yaws = []

        self._feedback.second_trap_stage = False
        self._as.publish_feedback(self._feedback)

        if goal.destination == goal.AMP:
            rospy.loginfo("2024_align_to_trap: Aligning to amp")
            amp_tags = self.RED_AMP if self.color == MatchSpecificData.ALLIANCE_COLOR_RED else self.BLUE_AMP
            while not rospy.is_shutdown():
                # check that preempt has not been requested by the client
                if self._as.is_preempt_requested():
                    rospy.logerr('%s: Preempted' % self._action_name)
                    self._as.set_preempted()
                    self.drive_to_object_client.cancel_goals_at_and_before_time(rospy.Time.now())
                    success = False
                    self._result.success = False
                    return   
                rate.sleep()
                rospy.loginfo_throttle(0.5, "align_to_trap: waiting for amp tag to become visible")
                visible_amp_tags = list(filter(lambda tag: f"tag_{tag}" in self.visible_objects, amp_tags))
                rospy.loginfo(f"visible_amp_tags {visible_amp_tags}")
                rospy.loginfo(f"self.visible_amp_tags {self.visible_objects}")
                rospy.loginfo(f"amp_tags {amp_tags}")
                if len(visible_amp_tags) > 0:
                    closest_tag = visible_amp_tags[0]
                    break
            yaws = self.AMP_YAW
            self.x_tolerance = self.amp_x_tolerance
            self.y_tolerance = self.amp_y_tolerance
            self.angle_tolerance = self.amp_angle_tolerance
            self.min_x_vel = self.amp_min_x_vel
            self.min_y_vel = self.amp_min_y_vel
            self.fast_zone = self.amp_fast_zone
            self.frame = self.amp_frame

        elif goal.destination == goal.SUBWOOFER:
            rospy.loginfo("2024_align_to_trap: Aligning to subwoofer")
            subwoofer_tags = self.RED_SUBWOOFER if self.color == MatchSpecificData.ALLIANCE_COLOR_RED else self.BLUE_SUBWOOFER
            while not rospy.is_shutdown():
                # check that preempt has not been requested by the client
                if self._as.is_preempt_requested():
                    rospy.logerr('%s: Preempted' % self._action_name)
                    self._as.set_preempted()
                    self.drive_to_object_client.cancel_goals_at_and_before_time(rospy.Time.now())
                    success = False
                    self._result.success = False
                    return                    
                rate.sleep()
                rospy.loginfo_throttle(0.5, "align_to_trap: waiting for subwoofer tag to become visible")
                visible_subwoofer_tags = list(filter(lambda tag: f"tag_{tag}" in self.visible_objects, subwoofer_tags))
                if len(visible_subwoofer_tags) > 0:
                    closest_tag = visible_subwoofer_tags[0]
                    break
            yaws = self.RED_SUBWOOFER_YAW if self.color == MatchSpecificData.ALLIANCE_COLOR_RED else self.BLUE_SUBWOOFER_YAW
            self.x_tolerance = self.amp_x_tolerance
            self.y_tolerance = self.amp_y_tolerance
            self.angle_tolerance = self.amp_angle_tolerance
            self.min_x_vel = self.amp_min_x_vel
            self.min_y_vel = self.amp_min_y_vel
            self.fast_zone = self.amp_fast_zone
            self.frame = self.subwoofer_frame

        else:
            rospy.loginfo("2024_align_to_trap: Aligning to trap")
            trap_tags = self.RED_TAGS if self.color == MatchSpecificData.ALLIANCE_COLOR_RED else self.BLUE_TAGS
            while not rospy.is_shutdown():
                # check that preempt has not been requested by the client
                if self._as.is_preempt_requested():
                    rospy.logerr('%s: Preempted' % self._action_name)
                    self._as.set_preempted()
                    self.drive_to_object_client.cancel_goals_at_and_before_time(rospy.Time.now())
                    success = False
                    self._result.success = False
                    return                 
                rate.sleep()
                rospy.loginfo_throttle(0.5, "align_to_trap: waiting for trap tag to become visible")
                visible_trap_tags = list(filter(lambda tag: f"tag_{tag}" in self.visible_objects, trap_tags))
                if len(visible_trap_tags) > 0:
                    closest_tag = visible_trap_tags[0]
                    break
            yaws = self.RED_TRAP_YAW if self.color == MatchSpecificData.ALLIANCE_COLOR_RED else self.BLUE_TRAP_YAW
            self.x_tolerance = self.trap_x_tolerance
            self.y_tolerance = self.trap_y_tolerance
            self.angle_tolerance = self.trap_angle_tolerance
            self.min_x_vel = self.trap_min_x_vel
            self.min_y_vel = self.trap_min_y_vel
            self.fast_zone = self.trap_fast_zone
            self.frame = self.stage_1_trap_frame
        
        yaw = min(yaws, key=lambda y: abs(angles.shortest_angular_distance(self.current_yaw, y)))
        rospy.loginfo(f"current yaw {self.current_yaw}, yaws {yaws} selected {yaw}")

        drive_to_object_done = False

        def done_callback(state, result):
            nonlocal drive_to_object_done
            rospy.loginfo(f"Drive to object actionlib finished with state {state} and result {result}")
            drive_to_object_done = True

        drive_to_object_goal = behavior_actions.msg.DriveToObjectGoal()
        drive_to_object_goal.id = f"tag_{closest_tag}"
        drive_to_object_goal.x_tolerance = self.x_tolerance
        drive_to_object_goal.y_tolerance = self.y_tolerance
        drive_to_object_goal.transform_to_drive = self.frame
        drive_to_object_goal.use_y = True
        drive_to_object_goal.min_x_vel = self.min_x_vel
        drive_to_object_goal.min_y_vel = self.min_y_vel
        drive_to_object_goal.override_goal_angle = False
        drive_to_object_goal.field_relative_angle = yaw
        drive_to_object_goal.fast_zone = self.fast_zone
        self.drive_to_object_client.send_goal(drive_to_object_goal, done_cb=done_callback)

        while not rospy.is_shutdown():
            # check that preempt has not been requested by the client
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                self.drive_to_object_client.cancel_goals_at_and_before_time(rospy.Time.now())
                success = False
                self._result.success = False
                break
            if drive_to_object_done:
                if goal.destination == goal.TRAP and not stage_2_trap:
                    self._feedback.second_trap_stage = True
                    self._as.publish_feedback(self._feedback)
                    rospy.loginfo("Moving to stage 2 trap aligment")
                    drive_to_object_done = False
                    stage_2_trap = True
                    drive_to_object_goal = behavior_actions.msg.DriveToObjectGoal()
                    drive_to_object_goal.id = f"tag_{closest_tag}"
                    drive_to_object_goal.x_tolerance = 0.03
                    drive_to_object_goal.y_tolerance = 0.03
                    drive_to_object_goal.transform_to_drive = self.stage_2_trap_frame
                    drive_to_object_goal.use_y = True
                    drive_to_object_goal.min_x_vel = self.min_x_vel
                    drive_to_object_goal.min_y_vel = self.min_y_vel
                    drive_to_object_goal.field_relative_angle = yaw
                    drive_to_object_goal.fast_zone = self.fast_zone
                    self.drive_to_object_client.send_goal(drive_to_object_goal, done_cb=done_callback)
                    continue
                elif goal.destination == goal.TRAP and stage_2_trap:
                    rospy.loginfo("Align to trap: stage 2 trap complete")
                success = self.drive_to_object_client.get_result().success
                break
            rate.sleep()
        if success:
            self._result.success = True
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)
        
if __name__ == '__main__':
    rospy.init_node('aligner')
    server = Aligner(rospy.get_name())
    rospy.spin()