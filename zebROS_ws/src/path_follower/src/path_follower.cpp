/* 1. information you have: current position and velcocity of the robot, waypoints already limited for velocity and acceleration
 * 2. find path point closest to the position of the robot
 * 3. find the goal point -- that is, point which the robot should drive to (based on lookahead distance)
 */
#include "path_follower/path_follower.h"
#include "ros/ros.h"

bool PathFollower::loadPath(const nav_msgs::Path &path)
{
	path_ = path;
	num_waypoints_ = path_.poses.size();
	if (num_waypoints_ == 0)
	{
		return false;
	}
	path_length_ = 0;
	vec_path_length_.push_back(0); // Path length at the first waypoint is 0
	ROS_INFO_STREAM("0: (" << path_.poses[0].pose.position.x << ", " << path_.poses[0].pose.position.y << ")");
	for (size_t i = 0; i < num_waypoints_ - 1; i++)
	{
		ROS_INFO_STREAM(i << ": (" << path_.poses[i + 1].pose.position.x << ", " << path_.poses[i + 1].pose.position.y << ", " << getYaw(path_.poses[i + 1].pose.orientation) << ")");
		double start_x = path_.poses[i].pose.position.x;
		double start_y = path_.poses[i].pose.position.y;
		double end_x = path_.poses[i + 1].pose.position.x;
		double end_y = path_.poses[i + 1].pose.position.y;

		path_length_ += hypot(end_x - start_x, end_y - start_y);
		vec_path_length_.push_back(path_length_);
	}
	return true;
}

const double PathFollower::getPathLength()
{
	return path_length_;
}

double PathFollower::getYaw(const geometry_msgs::Quaternion q)
{
	double roll, pitch, yaw;
	tf2::Quaternion tf_q(
		q.x,
		q.y,
		q.z,
		q.w);
	tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
	return yaw;
}

// The idea would be to have other code be responsible for getting current
// position and passing it in to run. run would then return a pose (x_pos, y_pos,
// orientation) and whoever called run would be responsible for sending that
// where it needs to go.
geometry_msgs::Pose PathFollower::run(nav_msgs::Odometry odom, double &total_distance_travelled)
{
	ROS_INFO_STREAM("----------------------------------------------");
	ROS_INFO_STREAM("current_position = " << odom.pose.pose.position.x
					<< " " << odom.pose.pose.position.y);

	double current_x = odom.pose.pose.position.x;
	double current_y = odom.pose.pose.position.y;
	double current_x_path, current_y_path;
	size_t current_waypoint_index = 0; //the index BEFORE the point on the path
	double minimum_distance = std::numeric_limits<double>::max();

	double start_x;
	double start_y;
	double end_x;
	double end_y;
	double dx;
	double dy;

	double magnitude_projection; // distance from the waypoint to the point on the path
	double distance_to_travel = 0; // distance from the point on the path, along the path
	if (num_waypoints_ == 0)
	{
		ROS_ERROR_STREAM("No waypoints in path");
		geometry_msgs::Pose target_pos;
		target_pos.position.x = 0;
		target_pos.position.y = 0;
		target_pos.position.z = 0;
		target_pos.orientation.x = 0;
		target_pos.orientation.y = 0;
		target_pos.orientation.z = 0;
		target_pos.orientation.w = 1;
		return target_pos; //TODO: better way to handle errors? This will just time out the server
	}
	int last_index = num_waypoints_ - 1;

	// Find point in path closest to odometry reading
	ROS_INFO_STREAM("num_waypoints = " << num_waypoints_);
	for (size_t i = 0; i < last_index; i++)
	{
		ROS_INFO_STREAM("test_point = " << path_.poses[i].pose.position.x << ", " << path_.poses[i].pose.position.y << ", " << getYaw(path_.poses[i].pose.orientation));
		// The line segment between this waypoint and the next waypoint
		start_x = path_.poses[i].pose.position.x;
		start_y = path_.poses[i].pose.position.y;
		end_x = path_.poses[i + 1].pose.position.x;
		end_y = path_.poses[i + 1].pose.position.y;

		dx = end_x - start_x;
		dy = end_y - start_y;
		double distance_between_waypoints_square = dx * dx + dy * dy;
		double innerProduct = (current_x - start_x) * dx + (current_y - start_y) * dy;
		// If the current position is normal to this segment
		if (0 <= innerProduct && innerProduct <= distance_between_waypoints_square)
		{
			double distance_between_waypoints = sqrt(distance_between_waypoints_square);
			// Find location of projection onto path
			// Distance from waypoint to projection onto path
			magnitude_projection = innerProduct / distance_between_waypoints;
			// Find path location
			current_x_path = start_x + magnitude_projection * (dx / distance_between_waypoints);
			current_y_path = start_y + magnitude_projection * (dy / distance_between_waypoints);
			ROS_INFO_STREAM("current path point = " << current_x_path << ", " << current_y_path);

			// Update minimum distance from path
			double distance_from_path = hypot(current_x - current_x_path, current_y - current_y_path);
			ROS_INFO_STREAM("distance_from_path = " << distance_from_path);
			if (distance_from_path < minimum_distance)
			{
				minimum_distance = distance_from_path;
				current_waypoint_index = i;
				ROS_INFO_STREAM("dx, dy = " << dx << ", " << dy << "; magnitude_projection = " << magnitude_projection);
				distance_to_travel = distance_between_waypoints - magnitude_projection; // Current value is distance that the robot will travel to reach the next waypoint
			}
		}
	}

	// if you're farther away from the path than the length of the path
	// or if you're not normal to any segment of the path
	// go to beginning / end of path (whichever is fastest)
	if (minimum_distance > path_length_)
	{
		double dist_from_startpoint = hypot(current_x -  path_.poses[0].pose.position.x, current_y - path_.poses[0].pose.position.y);
		double dist_from_endpoint = hypot(current_x -  path_.poses[last_index].pose.position.x, current_y - path_.poses[last_index].pose.position.y);
		geometry_msgs::Pose target_pos;
		if (dist_from_startpoint < dist_from_endpoint)
		{
			if (dist_from_startpoint < start_point_radius_)
			{
				ROS_INFO_STREAM("Robot is close enough to start_point to drive");
				current_waypoint_index = 0;

				//set distance_to_travel equal to the length of the first segment
				start_x = path_.poses[0].pose.position.x;
				start_y = path_.poses[0].pose.position.y;
				end_x = path_.poses[1].pose.position.x;
				end_y = path_.poses[1].pose.position.y;
				dx = end_x - start_x;
				dy = end_y - start_y;
				distance_to_travel = hypot(dx, dy);
			}
			else
			{
				ROS_ERROR_STREAM("Not within path limits : driving to start waypoint");
				target_pos.position.x = path_.poses[0].pose.position.x;
				target_pos.position.y = path_.poses[0].pose.position.y;
				target_pos.position.z = 0;
				target_pos.orientation = path_.poses[0].pose.orientation;
				return target_pos;
			}
		}
		else
		{
			ROS_ERROR_STREAM("Not within path limits : driving to end waypoint");
			target_pos.position.x = path_.poses[last_index].pose.position.x;
			target_pos.position.y = path_.poses[last_index].pose.position.y;
			target_pos.position.z = 0;
			target_pos.orientation = path_.poses[last_index].pose.orientation;
			return target_pos;
		}
	}

	ROS_INFO_STREAM("minimum_distance = " << minimum_distance);

	// Determine next waypoint
	size_t end_i = current_waypoint_index + 1; // the waypoint after the point normal to the position of the robot
	for (; end_i < last_index; end_i++)
	{
		ROS_INFO_STREAM("index = " << end_i << ", distance_to_travel = " << distance_to_travel);
		start_x = path_.poses[end_i].pose.position.x;
		start_y = path_.poses[end_i].pose.position.y;
		end_x = path_.poses[end_i + 1].pose.position.x;
		end_y = path_.poses[end_i + 1].pose.position.y;
		double dist = hypot(end_x - start_x, end_y - start_y);

		if (distance_to_travel + dist < lookahead_distance_)
			distance_to_travel += dist;
		else
			break;
	}
	ROS_INFO_STREAM("index before end point: " << end_i);

	double final_x, final_y, final_orientation;
	if (end_i == last_index) // if the lookahead distance is after the path has ended
	{
		ROS_INFO_STREAM("Current index is " << end_i << "; going to the end of the path");
		final_x = path_.poses[last_index].pose.position.x;
		final_y = path_.poses[last_index].pose.position.y;
		final_orientation = getYaw(path_.poses[last_index].pose.orientation);
	}
	else //if we're still in the path
	{
		ROS_INFO_STREAM("Current index is " << end_i << "; going to point within the path");
		ROS_INFO_STREAM("distance already travelled through waypoints = " << distance_to_travel);
		ROS_INFO_STREAM("lookahead distance = " << lookahead_distance_);
		// Add fraction of distance between waypoints to find final x and y
		dx = path_.poses[end_i + 1].pose.position.x - path_.poses[end_i].pose.position.x;
		dy = path_.poses[end_i + 1].pose.position.y - path_.poses[end_i].pose.position.y;
		double distance_between_waypoints = hypot(dx, dy);
		final_x = path_.poses[end_i].pose.position.x + (lookahead_distance_ - distance_to_travel) * (dx / distance_between_waypoints);
		final_y = path_.poses[end_i].pose.position.y + (lookahead_distance_ - distance_to_travel) * (dy / distance_between_waypoints);

		// Determine target orientation
		double start_yaw = getYaw(path_.poses[end_i].pose.orientation);
		double end_yaw = getYaw(path_.poses[end_i + 1].pose.orientation);
		ROS_INFO_STREAM("start yaw = " << start_yaw << ", end_yaw = " << end_yaw << ", dx = " << dx << ", dy = " << dy);
		final_orientation = start_yaw + (end_yaw - start_yaw) * ((lookahead_distance_ - distance_to_travel) / distance_between_waypoints);
	}
	ROS_INFO_STREAM("drive to coordinates: (" << final_x << ", " << final_y << ")");
	ROS_INFO_STREAM("drive to orientation: " << final_orientation);

	// Convert back to quaternion
	tf2::Quaternion q_final_tf = tf2::Quaternion(tf2Scalar(0), tf2Scalar(0), tf2Scalar(final_orientation));
	geometry_msgs::Quaternion q_final = tf2::toMsg(q_final_tf);

	// Return Pose of target position
	geometry_msgs::Pose target_pos;
	target_pos.position.x = final_x;
	target_pos.position.y = final_y;
	target_pos.position.z = 0;
	target_pos.orientation = q_final;

	total_distance_travelled = vec_path_length_[current_waypoint_index] + magnitude_projection;

	ROS_INFO_STREAM("Successfully returned target position and orientation");
	return target_pos;
}
