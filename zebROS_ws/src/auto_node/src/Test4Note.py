from auto_base import AutoBase
from series_action import SeriesAction
from wait_action import WaitAction
from drive_trajectory_action import DriveTrajectoryAction # REPLACE WITH ITERATOR WHEN FINISHED

class Test4Note(AutoBase):
    def __init__(self) -> None:
        super().__init__(display_name="Test4Note", # must match choreo path name
                         expected_trajectory_count=3) # how many segments of the path there are (split at waypoints)

    def get_action(self) -> SeriesAction:
        return SeriesAction([
            WaitAction(2),
            DriveTrajectoryAction(self.get_display_name(), 0),
            DriveTrajectoryAction(self.get_display_name(), 1),
            DriveTrajectoryAction(self.get_display_name(), 2)
        ])
