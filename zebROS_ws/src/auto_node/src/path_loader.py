# loads .csv files for the current auto and publishes them to /auto/current_auto_path where the trajectory action can use it
import rospy
import csv
import tf2_ros
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion
import os
from frc_utils.match_data_helper import RobotStatusHelper, Alliance, RobotMode

class PathLoader:
    def __init__(self):
        self.premade_position_paths_ = {}
        self.premade_velocity_paths_ = {}
        self.premade_position_waypoints_ = {}
        self.premade_velocity_waypoints_ = {}
        self.waypointsIdxs_ = {}
        self.__robot_status = RobotStatusHelper() 

        self.auto_name = None
        self.first_point_pub_ = rospy.Publisher('/first_point', PoseStamped, queue_size=10)
        self.latched_path_pub_ = rospy.Publisher( latch=True)
        self.old_alliance: Alliance = Alliance.UNKNOWN
        self.old_auto_name = ""
        self.timer = rospy.Timer(rospy.Duration(1), self.__load_path)

    # sets the auto that paths will be loaded for. Respects changes in alliance color. 
    def set_auto_name(self, auto_name: str):
        rospy.loginfo(f"Set path loader auto string to {auto_name}")

    def __load_path(self, _):
        alliance : Alliance = self.__robot_status.get_alliance()
        if alliance == Alliance.UNKNOWN:
            rospy.logwarn("Alliance is unknown in the path loader!")
            return

        # nothing has changed so can leave early
        if self.old_alliance == alliance and self.old_auto_name == self.auto_name:
            return
        self.old_alliance = alliance
        self.old_auto_name = self.auto_name

        rospy.loginfo(f"Publishing {self.auto_name}'s path for the {alliance} alliance")
        file_path = os.path.join("/home/ubuntu/2023RobotCode/zebROS_ws/src/behaviors/path/", f"{self.auto_steps_[j]}.csv")
        path = []

        with open(file_path, 'r') as csvfile:
            reader = csv.reader(csvfile)
            for row in reader:
                doubles = list(map(float, row))
                path.append(doubles)

        pos_path_msg = Path()
        pos_path_msg.header.frame_id = "map"
        pos_path_msg.header.stamp = rospy.Time(0)

        vel_path_msg = Path()
        vel_path_msg.header.frame_id = "map"
        vel_path_msg.header.stamp = rospy.Time(0)
        
        waypoints_idx = []
        for i, point in enumerate(path):
            pose = PoseStamped()
            pose.header.frame_id = "map" 
            pose.pose.position.x = point[1]
            pose.pose.position.y = point[2]
            quaternion = tf2_ros.transformations.quaternion_from_euler(0, 0, point[3])
            pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = quaternion
            pose.header.stamp = rospy.Time(point[0])
            pose.header.seq = i
            pos_path_msg.poses.append(pose)

            vel_pose = PoseStamped()
            vel_pose.header.frame_id = "map"
            vel_pose.pose.position.x = point[5]
            vel_pose.pose.position.y = point[6]
            vel_quaternion = tf2_ros.transformations.quaternion_from_euler(0, 0, point[4])
            vel_pose.pose.orientation.x, vel_pose.pose.orientation.y, vel_pose.pose.orientation.z, vel_pose.pose.orientation.w = vel_quaternion
            vel_pose.header.stamp = rospy.Time(point[0])
            vel_pose.header.seq = i
            vel_path_msg.poses.append(vel_pose)

            waypoints_idx.append(int(point[7]))

        self.premade_position_paths_[self.auto_steps_[j]] = pos_path_msg
        self.premade_velocity_paths_[self.auto_steps_[j]] = vel_path_msg
        self.premade_position_waypoints_[self.auto_steps_[j]] = Path()
        self.premade_velocity_waypoints_[self.auto_steps_[j]] = Path()
        self.waypointsIdxs_[self.auto_steps_[j]] = waypoints_idx
        
        if len(self.premade_position_paths_) == 1:
            self.first_point_pub_.publish(pos_path_msg.poses[0].pose)
