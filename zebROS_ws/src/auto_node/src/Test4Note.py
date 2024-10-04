from auto_base import AutoBase
from series_action import SeriesAction
from wait_action import WaitAction
from drive_trajectory_iterator import DriveTrajectoryActionIterator
from parallel_action import ParallelAction
from intake_action import IntakeAction

class Test4Note(AutoBase):
    def __init__(self) -> None:
        super().__init__(display_name="2025_4_Note", # must match choreo path name
                         expected_trajectory_count=3) # how many segments of the path there are (split at waypoints)

    def get_action(self) -> SeriesAction:
        drive_traj_iter = DriveTrajectoryActionIterator(self.get_display_name(), self.expected_trajectory_count)
        
        return SeriesAction([
            IntakeAction(off=False), 
            drive_traj_iter.get_next_trajectory_action(),
            drive_traj_iter.get_next_trajectory_action(),
            drive_traj_iter.get_next_trajectory_action(),
            WaitAction(2),
            IntakeAction(off=True)
        ])
