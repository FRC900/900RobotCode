#!/usr/bin/env python3

# loads .csv files for the current auto and publishes them to /auto/current_auto_path where the trajectory action can use it
import rospy
import csv
import tf2_ros
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import os
from frc_utils.match_data_helper import RobotStatusHelper, Alliance, RobotMode
import re
from auto_node_msgs.msg import PathGoalArray 
from path_follower_msgs.msg import PathGoal

class PathLoader:
    def __init__(self):
        self.premade_position_paths_ = {}
        self.premade_velocity_paths_ = {}
        self.premade_position_waypoints_ = {}
        self.premade_velocity_waypoints_ = {}
        self.waypointsIdxs_ = {}
        self.__robot_status = RobotStatusHelper() 

        self.auto_name: str = None
        self.__latched_path_pub = rospy.Publisher("/auto/current_auto_path", PathGoalArray, tcp_nodelay=True, latch=True, queue_size=1)
        self.old_alliance: Alliance = Alliance.UNKNOWN
        self.old_auto_name: str = ""
        self.timer = rospy.Timer(rospy.Duration(1), self.__load_path)

    # sets the auto that paths will be loaded for. Respects changes in alliance color. 
    def set_auto_name(self, auto_name: str):
        rospy.loginfo_once(f"Set path loader auto string to {auto_name}")
        self.auto_name = auto_name
        self.__load_path("")

    def __load_path(self, _):
        # alliance.name.lower() returns red/blue
        alliance : Alliance = self.__robot_status.get_alliance()
        if alliance == Alliance.UNKNOWN:
            rospy.logwarn("Alliance is unknown in the path loader!")
            return
        if self.auto_name is None:
            rospy.logwarn_throttle(4, "Auto name is none is the path loader! (May be ok if auto has no path)")
            return

        # nothing has changed so can leave early
        if self.old_alliance == alliance and self.old_auto_name == self.auto_name:
            return
        self.old_alliance = alliance
        self.old_auto_name = self.auto_name


        rospy.loginfo(f"Publishing {self.auto_name}'s path for the {alliance.name} alliance")
        os.chdir("/home/ubuntu/900RobotCode/zebROS_ws/src/auto_node/paths")
        files = os.listdir(".")
        #print(f"Files {files}")

        # matches auto name followed by digit then alliance.csv
        # if we have more than 9 waypoints then we messed up somewhere else
        # if auto was Test4Note and alliance red
        # input ['Test4Note2_blue.csv', 'Test4Note3_blue.csv', 'Test4Note.traj', 'Test4Note_blue.csv', 'Test4Note3_red.csv', 'Test4Note.3.traj', 'Test4Note1_red.csv', 'Test4Note_red.csv', 'Test4Note.1.traj', 'Test4Note.2.traj', 'Test4Note1_blue.csv', 'Test4Note2_red.csv']
        # output ['Test4Note1_red.csv', 'Test4Note2_red.csv', 'Test4Note3_red.csv']

        pattern = fr"^{self.auto_name}\d+_({alliance.name.lower()}).csv$"
        matched_files = [file for file in files if re.match(pattern, file)]
        matched_files.sort()

        path_array = PathGoalArray() # contains all the paths needed to run for the auto

        for file in matched_files:
            path = []
            with open(file, 'r') as csvfile:
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
            # CSV format:
            # 0    1 2 3   4                5    6    7
            # time,x,y,yaw,angular_velocity,xvel,yvel,waypointIdx
            for i, point in enumerate(path):
                pose = PoseStamped()
                pose.header.frame_id = "map" 
                pose.pose.position.x = point[1]
                pose.pose.position.y = point[2]
                quaternion = quaternion_from_euler(0, 0, point[3])
                pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = quaternion
                pose.header.stamp = rospy.Time(point[0])
                pose.header.seq = i
                pos_path_msg.poses.append(pose)

                vel_pose = PoseStamped()
                vel_pose.header.frame_id = "map"
                vel_pose.pose.position.x = point[5]
                vel_pose.pose.position.y = point[6]
                vel_quaternion = quaternion_from_euler(0, 0, point[4])
                vel_pose.pose.orientation.x, vel_pose.pose.orientation.y, vel_pose.pose.orientation.z, vel_pose.pose.orientation.w = vel_quaternion
                vel_pose.header.stamp = rospy.Time(point[0])
                vel_pose.header.seq = i
                vel_path_msg.poses.append(vel_pose)

                waypoints_idx.append(int(point[7]))

            self.premade_position_paths_[self.auto_name] = pos_path_msg
            self.premade_velocity_paths_[self.auto_name] = vel_path_msg
            self.premade_position_waypoints_[self.auto_name] = Path()
            self.premade_velocity_waypoints_[self.auto_name] = Path()
            self.waypointsIdxs_[self.auto_name] = waypoints_idx
                        
            path_to_append = PathGoal()
            path_to_append.position_path = pos_path_msg
            path_to_append.position_waypoints = Path() # still don't quite know what these waypoints are for and I might have made them
            path_to_append.velocity_path = vel_path_msg
            path_to_append.velocity_waypoints = Path()
            path_to_append.waypointsIdx = waypoints_idx

            path_array.auto_name = self.auto_name
            path_array.path_segments.append(path_to_append)

        self.__latched_path_pub.publish(path_array)
        rospy.loginfo(f"Published path with {len(path_array.path_segments)} segments")

        # @TODO get this working again
        #if len(self.premade_position_paths_) == 1:
        #    self.first_point_pub_.publish(pos_path_msg.poses[0].pose)

# not normally meant to be run as main
if __name__ == "__main__":
    rospy.init_node("path_loader")
    path_loader = PathLoader()
    path_loader.set_auto_name("Test4Note")
    import time
    rospy.spin()