#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from tf.transformations import *
from math import radians
import actionlib
from path_follower_msgs.msg import *
import tf2_geometry_msgs
import geometry_msgs.msg as gmsg
from copy import deepcopy
from std_msgs.msg import Header
# std_msgs/Header header
#   uint32 seq
#   time stamp
#   string frame_id
# geometry_msgs/PoseStamped[] poses
#   std_msgs/Header header
#     uint32 seq
#     time stamp
#     string frame_id
#   geometry_msgs/Pose pose
#     geometry_msgs/Point position
#       float64 x
#       float64 y
#       float64 z
#     geometry_msgs/Quaternion orientation
#       float64 x
#       float64 y
#       float64 z
#       float64 w
# timeSeconds, xPositionMeters, yPositionMeters, headingDegrees, velocityMetersPerSecond, accelerationMetersPerSecondSq, curvatureRadPerMeter, holonomicRotationDegrees, angularVelocityDegreesPerSec, holonomicAngularVelocityDegreesPerSec
path = []
with open("/home/ubuntu/900RobotCode/zebROS_ws/src/pathplanner_ros/paths/PlacePickCone.csv", "r") as csv:
    for line in csv.readlines():
        if line[0] == '#':
            continue # comment
        path.append(list(map(float, line.split(",")))) # yay functional programming is cool!

rospy.init_node("path_planner_run", anonymous=True)

first = path[0]
tfs = gmsg.TransformStamped()
tfs.header.frame_id = "path"
tfs.child_frame_id = "base_link"
tfs.transform.rotation = gmsg.Quaternion(*quaternion_from_euler(0, 0, radians(first[7])))
tfs.transform.translation.x = first[1]
tfs.transform.translation.y = first[2]

ac = actionlib.SimpleActionClient("/path_follower/path_follower_server", PathAction)
#ac.wait_for_server()
print("Server exists!")

start = rospy.Time.now().to_sec()

path_msg = Path()
path_msg.header.frame_id = "base_link"
path_msg.header.stamp = rospy.Time.from_sec(start)

poses = []

for i in range(len(path)):
    point = path[i]
    pose = PoseStamped()
    pose.header.frame_id = "path"
    pose.pose.position.x = point[1]
    pose.pose.position.y = point[2]
    q = quaternion_from_euler(0, 0, radians(point[7]))
    pose.pose.orientation.x = q[0]
    pose.pose.orientation.y = q[1]
    pose.pose.orientation.z = q[2]
    pose.pose.orientation.w = q[3]
    pose_tf = tf2_geometry_msgs.do_transform_pose(pose, tfs)
    # ok we figured it out
    # def do_transform_pose(pose, transform):
    #     res.header = transform.header
    #     return res
    # so apparently res.header is just a pointer to transform.header????
    pose_tf.header = Header() # this fixes it
    pose_tf.header.stamp = rospy.Time.from_sec(start + point[0])
    pose_tf.header.seq = i
    pose_tf.header.frame_id = "base_link"
    path_msg.poses.append(pose_tf)
    # print(id(pose_tf.header))

# Creates a goal to send to the action server.
goal = PathGoal(path=path_msg, waypointsIdx=[0]*len(path_msg.poses))

print(path_msg)

# Sends the goal to the action server.
#ac.send_goal(goal)