from auto_base import AutoBase
from series_action import SeriesAction
from wait_action import WaitAction
from drive_trajectory_iterator import DriveTrajectoryActionIterator
from parallel_action import ParallelAction
from intake_action import IntakeAction
from dynamic_shoot_action import DynamicShootAction
from geometry_msgs.msg import Twist
from cmd_vel_action import CmdVelAction

class TestDriveRotate(AutoBase):
    def __init__(self) -> None:
        super().__init__(display_name="test_drive_rotate", # must match choreo path name (if expected traj count is != 0)
                         expected_trajectory_count=1) # how many segments of the path there are (split at waypoints)

    def get_action(self) -> SeriesAction:   
        drive_traj_iter = DriveTrajectoryActionIterator(self.get_display_name(), self.expected_trajectory_count)

        return SeriesAction([
            drive_traj_iter.get_next_trajectory_action()
        ])
