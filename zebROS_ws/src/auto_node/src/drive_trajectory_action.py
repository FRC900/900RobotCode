from action import Action
from frc_utils.match_data_helper import RobotStatusHelper
\
from typing import List
from path_follower_msgs.msg import PathAction, PathActionFeedback

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
