# Auto Node

## Refrences
[195 autos, check for how to compose actions](https://gitlab.team195.com/cyberknights/ros2/robots/knight_shift/autonomous_node/-/tree/main/autonomous_node/autos?ref_type=heads)

## How it works
There is a base class `action.py`, that "actions" are based off of. These actions are then run by `action_runner.py`. Actions are non blocking. The two foundational actions are `series_action.py` and `parallel_action.py`, which combined together make essentially a tree that should represent whatever we want to do in auto. This could also be used in teleop but is essentially just actionlib.

Actual autos all inherit from `auto_base.py`, which currently has just one important method `get_actions() -> SeriesAction`. This series action contains all the actions that are run in the auto (e.g waiting, pathing, intaking). The series action can (and will) contain other series and parallel actions to make more complex behaviors. However at the very minimum we can recreate autos just in series like the old auto node. 

**Important** to note that autos are completly coupled to the choreo path that matches there name. Choreo files are found in `paths/` and the .traj/.csv files are in `deploy/choreo`. The process for making a new path is fairly simple now, make it in choreo, export the .traj files **split at waypoints** (assuming you want to stop at various points in the auto) and rebuild `auto_node`. Rebuilding generated the needed _red/_blue csvs which is expected by `path_loader.py`. 
 
Pathing is somewhat a special case. The path is loaded and published (latching) to `/auto/current_auto_path`. The type is `auto_node_msgs/PathGoalArray.msg`. This msg contains an array of what is needed to make a `path_follower_msgs/Path.action` goal to send to the path follower. There are two actions relating to path following. `drive_trajectory_action.py` takes a trajectory index, which is then used to run the indexth path found on `/auto/current_auto_path`. `drive_trajectory_iterator.py` is the action actually called in autos and allows the next segment of a path to be run with `get_next_trajectory_action()`. 

## How to make Choreo paths
Open `choreo` in docker. Right now it is expected that all waypoints the "split" option is checked if it corresponds to a stop waypoint. Split or stop point on its own currently does nothing. There is other little bits of knowledge, probably the best advice for pathing is be humble and get something simple working and THEN speed it up.   

## Other Notes
Python packaging is annoying and the files should not all be flat in `src/`. At least an `autos/` directory for autos like `Test4Note.py` and `generic_actions` for the series/parallel/trajectory actions. Can be refactored in the future.

For ease of use and also because this is shamelessly copied from 195, there is a match data helper which can be used in python like 
``` python 
from frc_utils.match_data_helper import RobotStatusHelper, Alliance, RobotMode

...

alliance : Alliance = self.__robot_status.get_alliance()
if alliance == Alliance.UNKNOWN:
    rospy.logwarn("Alliance is unknown in the path loader!")
    return
```
which is pretty nice. 
