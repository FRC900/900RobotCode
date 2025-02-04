#!/usr/bin/env python3

import actionlib
import rospy
import math
import tf2_ros
import tf2_geometry_msgs
import std_msgs.msg
import sensor_msgs.msg
import math
import behavior_actions.msg
from tf.transformations import *
from frc_msgs.msg import MatchSpecificData
import norfair_ros.msg
import geometry_msgs.msg
import angles
from path_follower_msgs.msg import PathGoal, PathFeedback, PathResult, PathAction

# Choose closest reef face based on robot angle
# Then find tag location and drive to it using PID on x, y, angle
# ...wait we've done this before
# Could create a path with one point, then use map relative path follower
# May want to launch separate path follower to have separate PID values

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
        13: (0.851, 7.396) # 13
    }

    def __init__(self, name):   
        self._action_name = name

        self.x_tolerance = rospy.get_param("x_tolerance")
        self.y_tolerance = rospy.get_param("y_tolerance")
        self.angle_tolerance = rospy.get_param("angle_tolerance")
        
        self.color = 0
        self._as = actionlib.SimpleActionServer(self._action_name, behavior_actions.msg.AlignToReef2025Action, execute_cb=self.aligner_callback, auto_start = False)
        self._as.start()

        self.path_client = actionlib.SimpleActionClient("/path_follower/path_follower_server", PathAction)

        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)

        self.current_yaw = 0
        self.imu_subscribe = rospy.Subscriber("/imu/zeroed_imu", sensor_msgs.msg.Imu, self.imu_callback)

        self.team_subscribe = rospy.Subscriber("/frcrobot_rio/match_data", MatchSpecificData, self.match_data_callback)

        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)

    def match_data_callback(self, data_msg):
        self.color = data_msg.allianceColor

    def imu_callback(self, imu_msg):
        q = imu_msg.orientation
        euler = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.current_yaw = euler[2]

    def aligner_callback(self, goal: behavior_actions.msg.AlignToReef2025Goal):
        success = True
        rospy.loginfo(f"Auto Aligning Actionlib called with goal {goal}")
        rate = rospy.Rate(50.0)

        tags = self.BLUE_TAGS if self.color == MatchSpecificData.ALLIANCE_COLOR_BLUE else self.RED_TAGS
        yaw = min(tags.keys(), key=lambda y: abs(angles.shortest_angular_distance(self.current_yaw, math.radians(y))))
        rospy.loginfo(f"current yaw {self.current_yaw}, selected {yaw}")
        tag = tags[yaw]

        yaw = math.radians(yaw)
        rospy.loginfo(f"Yaw: {yaw}, tag: {tag}")

        # transform = self.t_buffer.lookup_transform("pipe", "base_link", rospy.Time())
        
        done = False

        def done_callback(state, result):
            nonlocal done
            rospy.loginfo(f"Path actionlib finished with state {state} and result {result}")
            done = True

        # We have the pose of the tag, but we want the necessary pose to align the robot to the reef
        # Apply the base_link -> {left/right}_pipe transform on top of the tag transform...or some ordering like that

        # This is the transform from map to tag (e.g. where tag is relative to map)
        # We want the transform from map to pipe (e.g. where pipe frame is relative to map)
        map_to_tag = geometry_msgs.msg.TransformStamped()
        map_to_tag.transform.translation.x = self.TAG_POS[tag][0]
        map_to_tag.transform.translation.y = self.TAG_POS[tag][1]
        map_to_tag.transform.translation.z = 0
        map_to_tag.transform.rotation.x, map_to_tag.transform.rotation.y, map_to_tag.transform.rotation.z, map_to_tag.transform.rotation.w = quaternion_from_euler(0, 0, yaw)

        map_to_tag_t, map_to_tag_r = translation_matrix([map_to_tag.transform.translation.x,map_to_tag.transform.translation.y,map_to_tag.transform.translation.z]), euler_matrix(0, 0, yaw)
        map_to_tag = concatenate_matrices(map_to_tag_t, map_to_tag_r)

        # So, invert the above transform (so we have where map is relative to tag)
        tag_to_map = inverse_matrix(map_to_tag)

        # pipe is the offset the robot should be in relative to the tag
        base_link_to_tag_wrt_pipe: geometry_msgs.msg.TransformStamped = self.tf_buffer.lookup_transform("base_link", "left_pipe" if goal.pipe == goal.LEFT_PIPE else "right_pipe", rospy.Time())
        base_link_to_tag_wrt_pipe_t, base_link_to_tag_wrt_pipe_r = translation_matrix([base_link_to_tag_wrt_pipe.transform.translation.x,base_link_to_tag_wrt_pipe.transform.translation.y,base_link_to_tag_wrt_pipe.transform.translation.z]), quaternion_matrix([base_link_to_tag_wrt_pipe.transform.rotation.x, base_link_to_tag_wrt_pipe.transform.rotation.y, base_link_to_tag_wrt_pipe.transform.rotation.z, base_link_to_tag_wrt_pipe.transform.rotation.w])
        base_link_to_tag_wrt_pipe = concatenate_matrices(base_link_to_tag_wrt_pipe_t, base_link_to_tag_wrt_pipe_r)

        # and apply the tag to pipe transform (aka just base_link to pipe)
        # which gives us pipe to map
        pipe_to_map = concatenate_matrices(base_link_to_tag_wrt_pipe, tag_to_map) # apparently concatenation works but * doesn't
        # and then invert again to get map to pipe
        map_to_pipe = inverse_matrix(pipe_to_map)
        scale, shear, euler, translation, perspective = decompose_matrix(map_to_pipe)
        print(scale, shear, euler, translation, perspective)

        # This is also (apply offset) `under` (inversion), effectively
        pipe_pose = geometry_msgs.msg.PoseStamped()
        pipe_pose.header.frame_id = "base_link"
        pipe_pose.pose.position.x, pipe_pose.pose.position.y, pipe_pose.pose.position.z = translation
        pipe_pose.pose.orientation.x, pipe_pose.pose.orientation.y, pipe_pose.pose.orientation.z, pipe_pose.pose.orientation.w = quaternion_from_euler(*euler)

        path_goal = PathGoal()
        path_goal.final_pos_tol = min(self.x_tolerance, self.y_tolerance)
        path_goal.final_rot_tol = self.angle_tolerance
        path_goal.position_path.poses.append(pipe_pose)
        path_goal.position_waypoints.poses.append(pipe_pose)
        path_goal.waypointsIdx.append(0)

        d = geometry_msgs.msg.PoseStamped()
        d.pose.orientation.w = 1 # bruh

        path_goal.velocity_path.poses.append(d)
        path_goal.velocity_waypoints.poses.append(d)

        rospy.loginfo(f"Sending path goal {path_goal}")
        self.path_client.send_goal(path_goal, done_cb=done_callback)

        while not rospy.is_shutdown():
            # check that preempt has not been requested by the client
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                self.path_client.cancel_goals_at_and_before_time(rospy.Time.now())
                success = False
                self._result.success = False
                break
            if done:
                success = self.path_client.get_result().success
                break

            # try:
                # trans = self.tf_buffer.lookup_transform('map', 'base_link', rospy.Time())
            # except:
                # rospy.loginfo("2025_align_to_reef_single_tag: Transform tree not up, no feedback this cycle")
                # rate.sleep()
                # continue
            # x_dist = trans.transform.translation.x
            # y_dist = trans.transform.translation.y
            # self._feedback.x_error = abs(x_dist-TAG_POS[tag][0])
            # self._feedback.y_error = abs(y_dist-TAG_POS[tag][1])
            # self._feedback.angle_error = abs(angles.shortest_angular_distance(self.current_yaw, yaw))
            self._as.publish_feedback(self._feedback)

            rate.sleep()
        if success:
            self._result.success = True
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)
        
if __name__ == '__main__':
    rospy.init_node('align_to_reef_tagslam')
    server = Aligner(rospy.get_name())
    rospy.spin()