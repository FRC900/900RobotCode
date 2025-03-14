import rospy
from action import Action
import actionlib.simple_action_client
from frc_utils.match_data_helper import RobotStatusHelper

from typing import List
from path_follower_msgs.msg import PathGoal, PathAction, PathFeedback, PathResult
from geometry_msgs.msg import PoseStamped
from auto_node_msgs.msg import PathGoalArray # have the path to be sent to the path follower
import actionlib

class DriveTrajectoryAction(Action):
    """An action that drives a trajectory and waits for completion before ending"""
    #TODO: Make these possibly class variables
    def __init__(self, autonomous_name : str, trajectory_index : int, expected_trajectory_count : int, dont_go_to_start: bool = False, final_pos_tol: float = None, final_rot_tol: float = None):
        self.__first_point_pub = rospy.Publisher("/auto/first_point", PoseStamped, queue_size=1, tcp_nodelay=True, latch=True)
        self.__path_follower_client = actionlib.SimpleActionClient("/path_follower/path_follower_server", PathAction)
        if not self.__path_follower_client.wait_for_server(rospy.Duration(5)):
            rospy.logerr("Path follower server not up after 5 seconds, exiting")
            exit(1)
        self.__current_path: PathGoalArray = None
        self.__autonomous_name = autonomous_name
        self.__trajectory_index = trajectory_index
        self.__expected_trajectory_count = expected_trajectory_count
        self.__latest_feedback: PathFeedback = None
        self.__path_sub = rospy.Subscriber("/auto/current_auto_path", PathGoalArray, self.path_sub, tcp_nodelay=True)
        self.__finished = False
        self.__dont_go_to_start = dont_go_to_start
        self.__final_pos_tol = final_pos_tol
        self.__final_rot_tol = final_rot_tol

    def path_sub(self, path_array: PathGoalArray):
        trajectory_count = len(path_array.path_segments)
        rospy.loginfo(f"Current path updated to path for {path_array.auto_name}")
        if self.__current_path is not None:
            rospy.logwarn("DriveTrajectoryAction - path updated when path was already set? Alliance color change?")
        
        if self.__expected_trajectory_count != trajectory_count:
            rospy.logerr(f"Expected trajectory count for {self.__autonomous_name} is not correct. Expecting {self.__expected_trajectory_count} but got {trajectory_count}. Not setting path")
            self.__current_path = None
            return

        self.__current_path = path_array

        if self.__trajectory_index == 0:
            try:
                first_pose = self.__current_path.path_segments[0].position_path.poses[0]
                self.__first_point_pub.publish(first_pose)
            except Exception as e:
                rospy.logwarn(f"drive_trajectory_action: failed to publish first waypoint with error {e}")

    def feedback_cb(self, msg: PathFeedback):
        # currently dont use feedback but might later
        self.__latest_feedback = msg

    def done_cb(self, status: PathFeedback, result: PathResult):
        rospy.loginfo(f"Pathing for step {self.__trajectory_index} DONE")
        self.__finished = True

    def start(self):
        # block until have an auto selected
        while self.__current_path is None:
            rospy.sleep(rospy.Duration(0.2))
            rospy.logwarn_throttle(2, "Blocking in DriveTrajectoryAction start until a path is loaded!")

        rospy.loginfo(f"Running path step {self.__trajectory_index} for auto {self.__autonomous_name}")
        path_follower_goal: PathGoal = PathGoal()
        # get the path segment to run and use it to fill in the path follower goal
        path_segment: PathGoal = self.__current_path.path_segments[self.__trajectory_index]
        
        path_follower_goal.position_path = path_segment.position_path
        path_follower_goal.position_waypoints = path_segment.position_waypoints
        path_follower_goal.velocity_path = path_segment.velocity_path
        path_follower_goal.velocity_waypoints = path_segment.velocity_waypoints
        path_follower_goal.waypointsIdx = path_segment.waypointsIdx

        if self.__dont_go_to_start:
            rospy.logwarn("Not going to start!")
            path_follower_goal.dont_go_to_start = self.__dont_go_to_start
        
        if self.__final_pos_tol != None:
            rospy.logwarn(f"Using non-default positional tolerance of {self.__final_pos_tol}")
            path_follower_goal.final_pos_tol = self.__final_pos_tol
        
        if self.__final_rot_tol != None:
            rospy.logwarn(f"Using non-default rotational tolerance of {self.__final_rot_tol}")
            path_follower_goal.final_rot_tol = self.__final_rot_tol

        self.__path_follower_client.send_goal(path_follower_goal, done_cb=self.done_cb, feedback_cb=self.feedback_cb)
        rospy.loginfo("Sent path follower goal!")

    def update(self):
        pass

    def done(self):
        pass

    def isFinished(self) -> bool:
        return self.__finished

    def preempt(self):
        rospy.logwarn("Preempt called for drive trajectory action, cancelling path goals")
        # will cancel a couple times when there are more than 1 drive trajectory actions but should be ok
        self.__path_follower_client.cancel_goals_at_and_before_time(rospy.Time.now()) 
        self.__finished = True

