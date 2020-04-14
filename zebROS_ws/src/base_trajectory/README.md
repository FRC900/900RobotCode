
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

	wheel_radius : robot's wheel radius

