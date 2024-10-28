from auto_base import AutoBase
from series_action import SeriesAction
from wait_action import WaitAction
from drive_trajectory_iterator import DriveTrajectoryActionIterator
from parallel_action import ParallelAction
from intake_action import IntakeAction
from dynamic_shoot_action import DynamicShootAction
from geometry_msgs.msg import Twist
from cmd_vel_action import CmdVelAction
import numpy as np

class TestCmdVelCircle(AutoBase):
    def __init__(self) -> None:
        super().__init__(display_name="TestCmdVelCircle", # must match choreo path name (if expected traj count is != 0)
                         expected_trajectory_count=0) # how many segments of the path there are (split at waypoints)

    def get_action(self) -> SeriesAction:
        # just to flex lol
        # try doing this with auto_mode_config.yaml :)

        theta = np.linspace(0, 2*np.pi, 120)
        x = np.cos(theta)
        y = np.sin(theta) 
        twists_to_run = []
        for x_speed,y_speed in zip(x,y):
            tw = Twist()
            tw.linear.x = x_speed * 5
            tw.linear.y = y_speed * 5
            twists_to_run.append(CmdVelAction(tw, 0.1))

        return SeriesAction(twists_to_run)
