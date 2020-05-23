
Package to generate spline-based trajectories for a robot drive base

Input is a set of waypoints - each waypoint is an x, y, theta coordinate to hit.
Input can also include a set of constraints. These define the coordinates of a bounding box with more strict kinematics than the default global ones
Output is a set of timestamped waypoints to drive. These waypoints will be optimized according to a set of kinematic constraint parameters.
Request and respone for this service call is defined in base\_trajectory\_msgs::GenerateSpline

Configuration parameters:

    seg_length_epsilon : maximum error for each segment when parameterizing spline arclength

	dist_between_arc_lengths : equal-spaced arc length distance. Used when processing velocity profile to pick intermediate points
	dist_between_arc_lengths_epsilon : error tolerance for dist_between_arc_length
	mid_time_inflation : multiplier to prev distance for picking midpoint of next distance searched in arc length subdivision.
	path_dist_between_arc_lengths : spacing of waypoints along final generated path
	path_dist_between_arc_lengths_epsilon : error tolerance for final path waypoint spacing

	initial_delta_cost_epsilon : RPROP initial deltaCost value
	min_delta_cost_epsilon : RPROP minimum deltaCost value
	initial_dparam : RPROP initial optimization value change

	path_distance_limit : how far robot can diverge from straight-line path between waypoints
	max_vel : max translational velocity
	max_linear_acc : max linear acceleration
	max_linear_dec : max linear deceleration
	max_cent_acc : max centrepital acceleration

	drive_base_radius : robot's drive base radius - half the distance of the diagonal between two opposite wheels"
Example Service Calls :

rosservice call /base_trajectory/spline_gen "points:
- positions: [0 , 0, 0]
  velocities: []
  accelerations: []
  effort: []
  time_from_start: {secs: 0, nsecs: 0}
- positions: [1, 3, 0]
  velocities: []
  accelerations: []
  effort: []
  time_from_start: {secs: 1, nsecs: 0}
- positions: [0, 6, 0]
  velocities: []
  accelerations: []
  effort: []
  time_from_start: {secs: 2, nsecs: 0}
  "

rosservice call /base_trajectory/spline_gen "points:
- positions: [0 , 0, 0]
  velocities: []
  accelerations: []
  effort: []
  time_from_start: {secs: 0, nsecs: 0}
- positions: [1, -0.05, 3]
  velocities: []
  accelerations: []
  effort: []
  time_from_start: {secs: 1, nsecs: 0}
- positions: [3, 1, 0]
  velocities: []
  accelerations: []
  effort: []
  time_from_start: {secs: 2, nsecs: 0}
- positions: [2, 3, 0]
  velocities: []
  accelerations: []
  effort: []
  time_from_start: {secs: 3, nsecs: 0}
- positions: [1, 1, 0]
  velocities: []
  accelerations: []
  effort: []
  time_from_start: {secs: 4, nsecs: 0}
- positions: [0, 2, 0]
  velocities: []
  accelerations: []
  effort: []
  time_from_start: {secs: 5, nsecs: 0}
- positions: [2, 2.5, 0]
  velocities: []
  accelerations: []
  effort: []
  time_from_start: {secs: 6, nsecs: 0}
  "

rosservice call /base_trajectory/spline_gen "points:
- positions: [0, 0, 0]
  velocities: []
  accelerations: []
  effort: []
  time_from_start: {secs: 0, nsecs: 0}
- positions: [0, 10, 0]
  velocities: []
  accelerations: []
  effort: []
  time_from_start: {secs: 1, nsecs: 0}

constraints:
- corner1: {x: -2.0, y: 3.0, z: 0.0}
  corner2: {x: 2.0, y: 2.0, z: 0.0}
  max_accel: 2.5
  max_decel: 2.5
  max_vel: 1.0
  max_cent_accel: 3.5
  path_limit_distance: 0.2"

