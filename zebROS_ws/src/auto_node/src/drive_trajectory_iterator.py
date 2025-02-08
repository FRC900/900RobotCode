import rospy
from drive_trajectory_action import DriveTrajectoryAction
from auto_node_msgs.msg import PathGoalArray # have the path to be sent to the path follower
from path_follower_msgs.msg import PathGoal

# notably NOT an action, just gets the right DriveTrajectoryAction to run and makes sure you don't mess up 
class DriveTrajectoryActionIterator():
    def __init__(self, autonomous_name : str, expected_trajectory_count : int) -> None:
        self.__trajectory_count = 0
        self.__autonomous_name = autonomous_name
        self.__trajectory_index_iterator = 0
    
        self.__expected_trajectory_count = expected_trajectory_count
        self.__current_path: PathGoalArray = None

        self.__path_sub = rospy.Subscriber("/auto/current_auto_path", PathGoalArray, self.path_sub, tcp_nodelay=True)

    def path_sub(self, path_array: PathGoalArray):
        self.__trajectory_count = len(path_array.path_segments)
        rospy.loginfo(f"Current path updated to path for {path_array.auto_name}")
        if self.__current_path is not None:
            rospy.logwarn("DriveTrajectoryIterator - path updated when path was already set? Should be Alliance color change?")
        
        if self.__expected_trajectory_count != self.__trajectory_count:
            rospy.logerr(f"Expected trajectory count for {self.__autonomous_name} is not correct. Expecting {self.__expected_trajectory_count} but got {self.__trajectory_count}. Not setting path")
            self.__current_path = None
            return 

        self.__current_path = path_array


    def get_next_trajectory_action(self) -> DriveTrajectoryAction:
        curr_iterator = self.__trajectory_index_iterator
        self.__trajectory_index_iterator = self.__trajectory_index_iterator + 1

        # if curr_iterator >= self.__trajectory_count:
        #     rospy.logerr(f"Index out of range for trajectory {curr_iterator} in auto {self.__autonomous_name}")
        #     return None
        
        return DriveTrajectoryAction(self.__autonomous_name, curr_iterator, self.__expected_trajectory_count)
    
    def reset_iterator(self):
        self.__trajectory_index_iterator = 0
