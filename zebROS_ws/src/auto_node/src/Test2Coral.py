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

class Test2Coral(AutoBase):
    def __init__(self) -> None:
        super().__init__(display_name="2025_2_Coral", # must match choreo path name
                         expected_trajectory_count=3) # how many segments of the path there are (split at waypoints)

    def get_action(self) -> SeriesAction:
        drive_traj_iter = DriveTrajectoryActionIterator(self.get_display_name(), self.expected_trajectory_count)
        
        return SeriesAction([
            drive_traj_iter.get_next_trajectory_action(),
            PlacingAction(),
            drive_traj_iter.get_next_trajectory_action(),
            WaitForIntakeAction(),
            drive_traj_iter.get_next_trajectory_action(),
            PlacingAction()
            ])