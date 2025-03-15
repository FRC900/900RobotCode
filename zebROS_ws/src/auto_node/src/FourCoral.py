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
from cmd_vel_action import CmdVelAction
from geometry_msgs.msg import Twist
class FourCoral(AutoBase):
    ELEVATOR_PERCENT = 0.4

    def __init__(self, name: str) -> None:
        super().__init__(display_name=name, # must match choreo path name
                         expected_trajectory_count=7) # how many segments of the path there are (split at waypoints)

    def get_action(self) -> SeriesAction:
        drive_traj_iter = DriveTrajectoryActionIterator(self.get_display_name(), self.expected_trajectory_count)

        tw = Twist()
        tw.linear.x = -0.5
        
        return SeriesAction([
            CmdVelAction(tw, 0.25),
            ParallelAction([
                drive_traj_iter.get_next_trajectory_action(dont_go_to_start=True),
                SeriesAction([WaitTrajectoryAction(self.ELEVATOR_PERCENT),
                              PlacingAction(True)
                              ])
            ]),
            PlacingAction(),
            WaitAction(0.5),
            drive_traj_iter.get_next_trajectory_action(),
            WaitForIntakeAction(),

            ParallelAction([
                drive_traj_iter.get_next_trajectory_action(),
                SeriesAction([WaitTrajectoryAction(self.ELEVATOR_PERCENT),
                              PlacingAction(True)
                              ])
            ]),
            PlacingAction(),
            WaitAction(0.5),
            drive_traj_iter.get_next_trajectory_action(),
            WaitForIntakeAction(),

            ParallelAction([
                drive_traj_iter.get_next_trajectory_action(),
                SeriesAction([WaitTrajectoryAction(self.ELEVATOR_PERCENT),
                              PlacingAction(True)
                              ])
            ]),
            PlacingAction(),
            WaitAction(0.5),
            drive_traj_iter.get_next_trajectory_action(),
            WaitForIntakeAction(),

            ParallelAction([
                drive_traj_iter.get_next_trajectory_action(),
                SeriesAction([WaitTrajectoryAction(self.ELEVATOR_PERCENT),
                              PlacingAction(True)
                              ])
            ]),
            PlacingAction(),
        ])
    
class FourCoralProcessor(FourCoral):
    def __init__(self):
        super().__init__(name="2025_4_Coral_Processor_Side")

class FourCoralNonProcessor(FourCoral):
    def __init__(self):
        super().__init__(name="2025_4_Coral")