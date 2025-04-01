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
import math

# NOTE: current updated path is ONLY for processor

class FourCoral(AutoBase):
    ELEVATOR_PERCENT_START = 0.4
    ELEVATOR_PERCENT = 0.7

    def __init__(self, name: str, do_push: bool = False) -> None:
        self.__do_push = do_push
        super().__init__(display_name=name, # must match choreo path name
                         expected_trajectory_count=7) # how many segments of the path there are (split at waypoints)

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
            # WaitAction(0.1),
            drive_traj_iter.get_next_trajectory_action(),
            WaitForIntakeAction(),

            ParallelAction([
                drive_traj_iter.get_next_trajectory_action(dont_go_to_start=True, enforce_actually_localized=True),
                SeriesAction([WaitTrajectoryAction(self.ELEVATOR_PERCENT),
                              PlacingAction(True)
                              ])
            ]),
            PlacingAction(),
            # WaitAction(0.1),
            drive_traj_iter.get_next_trajectory_action(),
            WaitForIntakeAction(),

            ParallelAction([
                drive_traj_iter.get_next_trajectory_action(dont_go_to_start=True, enforce_actually_localized=True),
                SeriesAction([WaitTrajectoryAction(self.ELEVATOR_PERCENT),
                              PlacingAction(True)
                              ])
            ]),
            PlacingAction(),
            # WaitAction(0.1),
            drive_traj_iter.get_next_trajectory_action(),
            WaitForIntakeAction(),

            ParallelAction([
                drive_traj_iter.get_next_trajectory_action(dont_go_to_start=True, enforce_actually_localized=True),
                SeriesAction([WaitTrajectoryAction(self.ELEVATOR_PERCENT),
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
    
class FourCoralProcessor(FourCoral):
    def __init__(self):
        super().__init__(name="2025_4_Coral_Processor_Side")

class PushFourCoral(FourCoral):
    def __init__(self):
        super().__init__(name="2025_Push_4_Coral", do_push=True)

class FourCoralNonProcessor(FourCoral):
    def __init__(self):
        super().__init__(name="2025_4_Coral")