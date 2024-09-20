import rospy

from drive_trajectory_action import DriveTrajectoryAction

class DriveTrajectoryActionIterator():
    def __init__(self, autonomous_name : str, expected_trajectory_count : int) -> None:
        self.__trajectory_count = 0
        self.__autonomous_name = autonomous_name
        self.__trajectory_index_iterator = 0
    
        self.__expected_trajectory_count = expected_trajectory_count

        self.get_auto_info_client = NodeHandle.node_handle.create_client(srv_type=GetAutonomousInfo, srv_name='/get_autonomous_info', qos_profile=rclpy.qos.qos_profile_services_default)
        while not self.get_auto_info_client.wait_for_service(timeout_sec=1.0):
            NodeHandle.node_handle.get_logger().info('Get Auto Info Service not available, waiting again...')

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
        
    def get_next_trajectory_action(self) -> DriveTrajectoryAction:
        curr_iterator = self.__trajectory_index_iterator
        self.__trajectory_index_iterator = self.__trajectory_index_iterator + 1

        if curr_iterator >= self.__trajectory_count:
            NodeHandle.node_handle.get_logger().error(f"Index out of range for trajectory {curr_iterator} in auto {self.__autonomous_name}")
            return None

        return DriveTrajectoryAction(self.__autonomous_name, curr_iterator)
    
    def reset_iterator(self):
        self.__trajectory_index_iterator = 0
