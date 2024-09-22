import rospy
from drive_trajectory_action import DriveTrajectoryAction

# notably NOT an action, just gets the right DriveTrajectoryAction to run 
class DriveTrajectoryActionIterator():
    def __init__(self, autonomous_name : str, expected_trajectory_count : int) -> None:
        self.__trajectory_count = 0
        self.__autonomous_name = autonomous_name
        self.__trajectory_index_iterator = 0
    
        self.__expected_trajectory_count = expected_trajectory_count

        rospy.Subscriber("/auto/", tcp_nodelay=True)
        try:
            if self.get_auto_info_client is not None:
                auto_info_req = GetAutonomousInfo.Request()
                auto_info_req.autonomous_name = self.__autonomous_name
                auto_info_response_future = self.get_auto_info_client.call_async(auto_info_req)

                rclpy.spin_until_future_complete(node=NodeHandle.node_handle, future=auto_info_response_future)

                if auto_info_response_future.done():
                    resp : GetAutonomousInfo.Response = auto_info_response_future.result()
                    self.__trajectory_count = resp.number_of_trajectories

                if self.__expected_trajectory_count != self.__trajectory_count:
                    NodeHandle.node_handle.get_logger().error(f"Expected trajectory count for {self.__autonomous_name} is not correct. Expecting {self.__expected_trajectory_count} but got {self.__trajectory_count}")

        except Exception as e:
            NodeHandle.node_handle.get_logger().error(f"Service call failed {e}")
    
    def 

    def get_next_trajectory_action(self) -> DriveTrajectoryAction:
        curr_iterator = self.__trajectory_index_iterator
        self.__trajectory_index_iterator = self.__trajectory_index_iterator + 1

        if curr_iterator >= self.__trajectory_count:
            NodeHandle.node_handle.get_logger().error(f"Index out of range for trajectory {curr_iterator} in auto {self.__autonomous_name}")
            return None

        return DriveTrajectoryAction(self.__autonomous_name, curr_iterator)
    
    def reset_iterator(self):
        self.__trajectory_index_iterator = 0
