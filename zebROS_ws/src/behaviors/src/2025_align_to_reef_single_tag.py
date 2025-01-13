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

    def __init__(self, name):   
        self._action_name = name

        self.x_tolerance = 0.1
        self.y_tolerance = 0.1
        self.angle_tolerance = 0.1
        self.min_x_vel = 1
        self.min_y_vel = 1
        self.fast_zone = 2
        # self.x_tolerance = rospy.get_param("reef_x_tolerance")
        # self.y_tolerance = rospy.get_param("reef_y_tolerance")
        # self.angle_tolerance = rospy.get_param("reef_angle_tolerance")
        # self.min_x_vel = rospy.get_param("reef_min_x_vel")
        # self.min_y_vel = rospy.get_param("reef_min_y_vel")
        # self.fast_zone = rospy.get_param("reef_fast_zone")
        
        self.color = 0
        self._as = actionlib.SimpleActionServer(self._action_name, behavior_actions.msg.AlignToReef2025Action, execute_cb=self.aligner_callback, auto_start = False)
        self._as.start()

        self.drive_to_object_client = actionlib.SimpleActionClient("/drive_to_object/drive_to_object", behavior_actions.msg.DriveToObjectAction)
        self.norfair_sub = rospy.Subscriber("/norfair/output", norfair_ros.msg.Detections, self.tracked_objects_callback)

        self.t_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.t_buffer)

        self.visible_objects = []
        
        self.current_yaw = 0
        self.orientation_command_pub = rospy.Publisher("/teleop/orientation_command", std_msgs.msg.Float64, queue_size=1)
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
        yaw = min(tags.keys(), key=lambda y: abs(angles.shortest_angular_distance(self.current_yaw * 180/math.pi, y)))
        tag = tags[yaw]

        transform = self.t_buffer.lookup_transform("pipe", "base_link", rospy.Time())
        
        drive_to_object_done = False

        def done_callback(state, result):
            nonlocal drive_to_object_done
            rospy.loginfo(f"Drive to object actionlib finished with state {state} and result {result}")
            drive_to_object_done = True

        drive_to_object_goal = behavior_actions.msg.DriveToObjectGoal()
        drive_to_object_goal.id = f"tag_{tag}"
        drive_to_object_goal.x_tolerance = self.x_tolerance
        drive_to_object_goal.y_tolerance = self.y_tolerance
        drive_to_object_goal.transform_to_drive = transform
        drive_to_object_goal.use_y = True
        drive_to_object_goal.min_x_vel = self.min_x_vel
        drive_to_object_goal.min_y_vel = self.min_y_vel
        drive_to_object_goal.override_goal_angle = False
        drive_to_object_goal.field_relative_angle = yaw
        drive_to_object_goal.fast_zone = self.fast_zone
        self.drive_to_object_client.send_goal(drive_to_object_goal, done_cb=done_callback)
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
            rate.sleep()
        if success:
            self._result.success = True
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)
        
if __name__ == '__main__':
    rospy.init_node('aligner')
    server = Aligner(rospy.get_name())
    rospy.spin()