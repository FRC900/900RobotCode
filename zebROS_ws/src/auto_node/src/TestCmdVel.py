from auto_base import AutoBase
from series_action import SeriesAction
from wait_action import WaitAction
from drive_trajectory_iterator import DriveTrajectoryActionIterator
from parallel_action import ParallelAction
from intake_action import IntakeAction
from dynamic_shoot_action import DynamicShootAction
from geometry_msgs.msg import Twist
from cmd_vel_action import CmdVelAction
from intake_action import IntakeAction

class TestCmdVel(AutoBase):
    def __init__(self) -> None:
        super().__init__(display_name="TestCmdVel", # must match choreo path name (if expected traj count is != 0)
                         expected_trajectory_count=0) # how many segments of the path there are (split at waypoints)

    def get_action(self) -> SeriesAction:   
        tw = Twist()        
        tw.linear.x = 1     
        tw.linear.y = 1     
        return SeriesAction([CmdVelAction(tw, 2)])
