import rospy
from action import Action
import actionlib.simple_action_client
from frc_utils.match_data_helper import RobotStatusHelper

from typing import List
from path_follower_msgs.msg import PathAction, PathFeedback, PathResult
from auto_node_msgs.msg import PathGoalArray, PathGoal # have the path to be sent to the path follower
import actionlib
from subsystem import Subsystem

class DriveTrajectoryAction(Action):
    """An action that drives a trajectory and waits for completion before ending"""
    #TODO: Make these possibly class variables
    def __init__(self, autonomous_name : str, trajectory_index : int):

        self.__path_follower_client = actionlib.SimpleActionClient("/path_follower/path_follower_server", PathAction)
        self.__current_path: PathGoalArray = None
        self.__autonomous_name = autonomous_name
        self.__trajectory_index = trajectory_index
        self.__latest_feedback: PathFeedback = None
        self.__path_sub = rospy.Subscriber("/auto/current_auto_path", PathGoalArray, self.path_sub, tcp_nodelay=True)
        self.__done = False

    def path_sub(self, path_array: PathGoalArray):
        rospy.loginfo(f"Current path updated to path for {path_array.auto_name}")
        if self.__current_path is not None:
            rospy.logwarn("DriveTrajectoryAction - path updated when path was already set? Alliance color change?")

        self.__current_path = path_array

    def feedback_cb(self, msg: PathFeedback):
        # currently dont use feedback but might later
        self.__latest_feedback = msg

    def done_cb(self, status: PathFeedback, result: PathResult):
        self.__done = True

    def start(self):
        rospy.loginfo(f"Running path step {self.__trajectory_index} for auto {self.__autonomous_name}")
        path_follower_goal: PathGoal = PathGoal()
        # get the path segment to run and use it to fill in the path follower goal
        path_segment: PathGoal = self.__current_path.path_segments[self.__trajectory_index]
        
        path_follower_goal.position_path = path_segment.position_path
        path_follower_goal.position_waypoints = path_segment.position_waypoints
        path_follower_goal.velocity_path = path_segment.velocity_path
        path_follower_goal.velocity_waypoints = path_segment.velocity_waypoints
        path_follower_goal.waypointsIdx = path_segment.waypointsIdx
        self.__path_follower_client.send_goal(path_follower_goal, done_cb=self.done_cb, feedback_cb=self.feedback_cb)
        rospy.loginfo("Sent path follower goal!")

    def update(self):
        pass

    def done(self):
        pass

    def isFinished(self) -> bool:
        return self.__done

    def affectedSystems(self) -> List[Subsystem]:
        return [ Subsystem.DRIVEBASE ]
