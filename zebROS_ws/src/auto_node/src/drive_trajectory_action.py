from actions_node.default_actions.action import Action
from frc_robot_utilities_py_node.frc_robot_utilities_py import FRCRobotUtils
from ck_ros2_base_msgs_node.msg import TrajectoryStatus
from typing import List
from actions_node.game_specific_actions.subsystem import Subsystem
from ck_utilities_py_node.node_handle import NodeHandle

from ck_ros2_base_msgs_node.srv import StartTrajectory

class DriveTrajectoryAction(Action):
    """An action that drives a trajectory and waits for completion before ending"""
    #TODO: Make these possibly class variables
    def __init__(self, autonomous_name : str, trajectory_index : int):
        self.__traj_status_subscriber = BufferedROSMsgHandlerPy(TrajectoryStatus)
        self.__traj_status_subscriber.register_for_updates("/TrajectoryStatus")
        self.__autonomous_name = autonomous_name
        self.__trajectory_index = trajectory_index

        self.start_traj_client = NodeHandle.node_handle.create_client(srv_type=StartTrajectory, srv_name='/start_trajectory', qos_profile=rclpy.qos.qos_profile_services_default, callback_group=rclpy.callback_groups.MutuallyExclusiveCallbackGroup())
        while not self.start_traj_client.wait_for_service(timeout_sec=1.0):
            NodeHandle.node_handle.get_logger().info('Start Traj Service not available, waiting again...')

    def start(self):
        start_traj_msg = StartTrajectory.Request()
        start_traj_msg.autonomous_name = self.__autonomous_name
        start_traj_msg.trajectory_index = self.__trajectory_index

        if (self.start_traj_client is not None):
            auto_run_response_future = self.start_traj_client.call_async(start_traj_msg)
            # if not auto_run_response.accepted:
            #     NodeHandle.node_handle.get_logger().error(f"Failed to start trajectory {self.__autonomous_name}: {self.__trajectory_index}")

    def update(self):
        #TODO: Possibly add retransmit logic on service call failure
        pass

    def done(self):
        pass

    def isFinished(self) -> bool:
        traj_status : TrajectoryStatus = self.__traj_status_subscriber.get()
        if traj_status is not None:
            return traj_status.is_completed and traj_status.trajectory_index == self.__trajectory_index
        return False

    def affectedSystems(self) -> List[Subsystem]:
        return [ Subsystem.DRIVEBASE ]
