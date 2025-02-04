#! /usr/bin/env python3

import actionlib
import rospy
import math
import tf2_ros
import tf2_geometry_msgs
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
    _feedback = behavior_actions.msg.AlignToReef2025Feedback()
    _result = behavior_actions.msg.AlignToReef2025Result()

    RED_TAGS = {
        120: 6,
        180: 7,
        240: 8,
        300: 9,
        0: 10,
        60: 11
    }
    BLUE_TAGS = {
        60: 17,
        0: 18,
        300: 19,
        240: 20,
        180: 21,
        120: 22
    }

    TAG_POS = {
    # tag id: (x, y)
        6: (13.474, 3.306), # 6
        7: (13.890, 4.026), # 7
        8: (13.474, 4.745), # 8
        9: (12.643, 4.745), # 9
        10: (12.227, 4.026), # 10
        11: (12.643, 3.306), # 11
        1: (16.697, 0.655), # 1
        2: (16.697, 7.396), # 2
        17: (4.047, 3.306), # 17
        18: (3.658, 4.026), # 18
        19: (4.074, 4.745), # 19
        20: (4.908, 4.745), # 20
        21: (5.321, 4.026), # 21
        22: (4.908, 3.306), # 22
        12: (0.851, 0.655), # 12
        13: (0.851, 7.396), # 13
    }

    def __init__(self, name):   
        self._action_name = name

        self.x_tolerance = rospy.get_param("x_tolerance")
        self.y_tolerance = rospy.get_param("y_tolerance")
        self.angle_tolerance = rospy.get_param("angle_tolerance")
        self.min_x_vel = rospy.get_param("min_x_vel")
        self.min_y_vel = rospy.get_param("min_y_vel")
        self.fast_zone = rospy.get_param("fast_zone")
        # self.x_tolerance = 0.1
        # self.y_tolerance = 0.1
        # self.angle_tolerance = 0.1
        # self.min_x_vel = 1
        # self.min_y_vel = 1
        # self.fast_zone = 2
        
        self.color = 0
        self._as = actionlib.SimpleActionServer(self._action_name, behavior_actions.msg.AlignToReef2025Action, execute_cb=self.aligner_callback, auto_start = False)
        self._as.start()

        self.drive_to_object_client = actionlib.SimpleActionClient("/drive_to_object/drive_to_object", behavior_actions.msg.DriveToObjectAction)
        self.norfair_sub = rospy.Subscriber("/norfair/output", norfair_ros.msg.Detections, self.tracked_objects_callback)

        self.t_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.t_buffer)

        self.visible_objects = []
        
        self.current_yaw = 0
        self.imu_subscribe = rospy.Subscriber("/imu/zeroed_imu", sensor_msgs.msg.Imu, self.imu_callback)

        self.team_subscribe = rospy.Subscriber("/frcrobot_rio/match_data", MatchSpecificData, self.match_data_callback)

    def match_data_callback(self, data_msg):
        self.color = data_msg.allianceColor

    def imu_callback(self, imu_msg):
        q = imu_msg.orientation
        euler = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.current_yaw = euler[2]

    def tracked_objects_callback(self, msg: norfair_ros.msg.Detections):
        self.visible_objects = [detection.label for detection in msg.detections]

    def aligner_callback(self, goal: behavior_actions.msg.AlignToReef2025Goal):
        success = True
        rospy.loginfo(f"Auto Aligning Actionlib called with goal {goal}")
        rate = rospy.Rate(50.0)
        stage_2_trap = False
        tags = self.BLUE_TAGS if self.color == MatchSpecificData.ALLIANCE_COLOR_BLUE else self.RED_TAGS
        yaw = min(tags.keys(), key=lambda y: abs(angles.shortest_angular_distance(self.current_yaw, math.radians(y))))
        rospy.loginfo(f"current yaw {self.current_yaw}, selected {yaw}")
        tag = tags[yaw]

        yaw = math.radians(yaw)
        rospy.loginfo(f"Yaw: {yaw}, tag: {tag}")

        # transform = self.t_buffer.lookup_transform("pipe", "base_link", rospy.Time())
        
        drive_to_object_done = False

        def done_callback(state, result):
            nonlocal drive_to_object_done
            rospy.loginfo(f"Drive to object actionlib finished with state {state} and result {result}")
            drive_to_object_done = True

        drive_to_object_feedback: behavior_actions.msg.DriveToObjectFeedback = None

        def feedback_callback(msg: behavior_actions.msg.DriveToObjectFeedback):
            nonlocal drive_to_object_feedback
            drive_to_object_feedback = msg

        drive_to_object_goal = behavior_actions.msg.DriveToObjectGoal()
        drive_to_object_goal.id = f"tag_{tag}"
        drive_to_object_goal.x_tolerance = self.x_tolerance
        drive_to_object_goal.y_tolerance = self.y_tolerance
        drive_to_object_goal.transform_to_drive = "left_pipe" if goal.pipe == goal.LEFT_PIPE else "right_pipe"
        drive_to_object_goal.use_y = True
        drive_to_object_goal.min_x_vel = self.min_x_vel
        drive_to_object_goal.min_y_vel = self.min_y_vel
        drive_to_object_goal.override_goal_angle = False
        drive_to_object_goal.field_relative_angle = yaw
        drive_to_object_goal.fast_zone = self.fast_zone
        self.drive_to_object_client.send_goal(drive_to_object_goal, done_cb=done_callback, feedback_cb=feedback_callback)
        rospy.loginfo("drive_to_object goal sent")

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
                success = self.drive_to_object_client.get_result().success
                break

            self._feedback.x_error = drive_to_object_feedback.x_error
            self._feedback.y_error = drive_to_object_feedback.y_error
            self._feedback.angle_error = drive_to_object_feedback.angle_error
            self._as.publish_feedback(self._feedback)

            rate.sleep()
        if success:
            self._result.success = True
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)

        
if __name__ == '__main__':
    rospy.init_node('align_to_reef_single_tag')
    server = Aligner(rospy.get_name())
    rospy.spin()