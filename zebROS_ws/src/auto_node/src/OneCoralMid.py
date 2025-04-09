from auto_base import AutoBase
from series_action import SeriesAction
from wait_action import WaitAction
from drive_trajectory_iterator import DriveTrajectoryActionIterator
from parallel_action import ParallelAction
#from intake_action import IntakeAction
from placing_action import PlacingAction
#from wait_intake_action import WaitIntakeAction
#from wait_trajectory_action import WaitTrajectoryAction
from wait_for_intake_action import WaitForIntakeAction
from wait_trajectory_action import WaitTrajectoryAction
from geometry_msgs.msg import Twist
from cmd_vel_action import CmdVelAction
from early_exit_parallel_action import EarlyExitParallelAction
import math

class OneCoralMid(AutoBase):
    ELEVATOR_PERCENT_START = 0.4
    ELEVATOR_PERCENT = 0.7

    def __init__(self, name: str, do_push: bool = False) -> None:
        self.__do_push = do_push
        super().__init__(display_name=name, # must match choreo path name
                         expected_trajectory_count=3) # how many segments of the path there are (split at waypoints)

    def get_action(self) -> SeriesAction:
        drive_traj_iter = DriveTrajectoryActionIterator(self.get_display_name(), self.expected_trajectory_count)
        tw = Twist()        
        tw.linear.x = -1     
        tw.linear.y = 0  
        
        actions = [
            ParallelAction([
                drive_traj_iter.get_next_trajectory_action(dont_go_to_start=True, enforce_actually_localized=True),
                SeriesAction([WaitTrajectoryAction(self.ELEVATOR_PERCENT_START),
                              PlacingAction(True)
                              ])
            ]),
            PlacingAction(),
        ]

        if self.__do_push:
            # NOTE THIS ONLY WORKS FOR NON PROCESSOR SIDE (our barge, which is the only place where we want to push an opponent anyway)
            # For processor side/opposite barge, y should be -0.5
            tw = Twist()
            tw.linear.y = 0.5
            actions = [CmdVelAction(twist=tw, time=0.5)] + actions
        
        return SeriesAction(actions)
    
class OneCoralMidProcessor(OneCoralMid):
    def __init__(self):
        super().__init__(name="2025_2_Mid_Coral")

class OneCoralMidNonProcessor(OneCoralMid):
    def __init__(self):
        super().__init__(name="2025_2_Mid_Coral_NonProcessor")