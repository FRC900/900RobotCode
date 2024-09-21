# same as the goal field for path_follower_msgs/Path.action, used to publish which path to run
nav_msgs/Path position_path
nav_msgs/Path position_waypoints
nav_msgs/Path velocity_path
nav_msgs/Path velocity_waypoints
int32[] waypointsIdx

# Linear and angular tolerances for finishing path
# Set to non-zero to override config defaults
float64 final_pos_tol
float64 final_rot_tol

bool dont_go_to_start