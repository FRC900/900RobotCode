/* 1. information you have: current position and velocity of the robot, waypoints already limited for velocity and acceleration
 * 2. find path point closest to the position of the robot
 * 3. find the goal point -- that is, point which the robot should drive to (based on time_offset setting to grab a target point in the future)
 */
#include "path_follower/path_follower.h"
#include "ros/ros.h"

// #define DEBUG

bool PathFollower::loadPath(const nav_msgs::Path &path, const nav_msgs::Path &velocities, double &final_distace)
{
	path_ = path;
	velocities_ = velocities;
	if (path_.poses.size() != velocities_.poses.size()) {
		return false;
	}
	num_waypoints_ = path_.poses.size();
	if (num_waypoints_ == 0)
	{
		return false;
	}
	path_length_ = 0;
	vec_path_length_.clear();
	vec_path_length_.push_back(0); // Path length at the first waypoint is 0
	for (size_t i = 0; i < num_waypoints_ - 1; i++)
	{
		ROS_INFO_STREAM(i << ": (" << path_.poses[i].header.stamp.toSec() << ", " << path_.poses[i + 1].pose.position.x << ", " << path_.poses[i + 1].pose.position.y << ", " << getYaw(path_.poses[i + 1].pose.orientation) << ")");
		const double start_x = path_.poses[i].pose.position.x;
		const double start_y = path_.poses[i].pose.position.y;
		const double end_x = path_.poses[i + 1].pose.position.x;
		const double end_y = path_.poses[i + 1].pose.position.y;

		path_length_ += hypot(end_x - start_x, end_y - start_y);
		vec_path_length_.push_back(path_length_);
		ROS_INFO_STREAM("ADDING " << path_length_ );
	}
	final_distace = vec_path_length_.back();
	start_time_offset_ = ros::Duration(ros::Time::now().toSec());
	return true;
}

double PathFollower::getPathLength() const
{
	return path_length_;
}

double PathFollower::getYaw(const geometry_msgs::Quaternion &q) const
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

double PathFollower::interpolate(double start_t, double end_t, double start_x, double end_x, double current_t) const
{
	if(current_t <= start_t)
		return start_x;
	if(current_t >= end_t)
		return end_x;
	return start_x + (end_x - start_x) / (end_t - start_t) * (current_t - start_t);
}

// The idea would be to have other code be responsible for getting current
// position and passing it in to run. run would then return a pose (x_pos, y_pos,
// orientation) and whoever called run would be responsible for sending that
// where it needs to go.

std::optional<PositionVelocity> PathFollower::run(double &total_distance_travelled, int &current_index)
{
	if (num_waypoints_ == 0)
	{
		ROS_ERROR_STREAM("No waypoints in path");
		return {};
	}

	// Timestamps in path are relative to when they were created.
	// Offset them here to be relative the time the path starts running
	const ros::Time current_time = ros::Time::now() - start_time_offset_ + ros::Duration(path_.poses[0].header.stamp.toSec());

	const size_t last_index = num_waypoints_ - 1;
	double final_x, final_y, final_x_velocity, final_y_velocity, final_orientation, final_velocities_orientation;
#ifdef DEBUG
	// Find point in path closest to odometry reading
	ROS_INFO_STREAM("num_waypoints = " << num_waypoints_);
#endif

	size_t index = 0; // index of point after current time
	size_t now_index = 0;
	for(; index < last_index; index++)
	{
		if(path_.poses[index].header.stamp < current_time + time_offset_)
			now_index += 1;
		if(path_.poses[index].header.stamp > current_time + time_offset_)
			break;
	}
	final_x = interpolate(path_.poses[index - 1].header.stamp.toSec(),
			path_.poses[index].header.stamp.toSec(),
			path_.poses[index - 1].pose.position.x,
			path_.poses[index].pose.position.x,
			current_time.toSec());
	final_y = interpolate(path_.poses[index - 1].header.stamp.toSec(),
			path_.poses[index].header.stamp.toSec(),
			path_.poses[index - 1].pose.position.y,
			path_.poses[index].pose.position.y,
			current_time.toSec());

	final_x_velocity = interpolate(velocities_.poses[index - 1].header.stamp.toSec(),
			velocities_.poses[index].header.stamp.toSec(),
			// The position here is actually a velocity 
			velocities_.poses[index - 1].pose.position.x, 
			velocities_.poses[index].pose.position.x,
			current_time.toSec());

	final_y_velocity = interpolate(velocities_.poses[index - 1].header.stamp.toSec(),
			velocities_.poses[index].header.stamp.toSec(),
			// The position here is actually a velocity 
			velocities_.poses[index - 1].pose.position.y, 
			velocities_.poses[index].pose.position.y,
			current_time.toSec());

	final_orientation = interpolate(path_.poses[index - 1].header.stamp.toSec(),
			path_.poses[index].header.stamp.toSec(),
			getYaw(path_.poses[index - 1].pose.orientation),
			getYaw(path_.poses[index].pose.orientation),
			current_time.toSec());
	
	final_velocities_orientation = interpolate(velocities_.poses[index - 1].header.stamp.toSec(),
			velocities_.poses[index].header.stamp.toSec(),
			getYaw(velocities_.poses[index - 1].pose.orientation),
			getYaw(velocities_.poses[index].pose.orientation),
			current_time.toSec());
#ifdef DEBUG
	ROS_INFO_STREAM("drive to coordinates: " << index << " (" << final_x << ", " << final_y << ", " << final_orientation << ")");
	ROS_INFO_STREAM("drive to velocity: " << index << " (" << final_x_velocity << ", " << final_y_velocity << ", " << final_velocities_orientation << ")");

	// This is strictly for the debugging ROS_INFO below
	double now_x;
	double now_y;
	double now_x_velocity;
	double now_y_velocity;
	double now_orientation;
	double now_velocities_orientation;
	if (now_index > 0)
	{
		now_x = interpolate(path_.poses[now_index - 1].header.stamp.toSec(),
				path_.poses[now_index].header.stamp.toSec(),
				path_.poses[now_index - 1].pose.position.x,
				path_.poses[now_index].pose.position.x,
				current_time.toSec());
		now_y = interpolate(path_.poses[now_index - 1].header.stamp.toSec(),
				path_.poses[now_index].header.stamp.toSec(),
				path_.poses[now_index - 1].pose.position.y,
				path_.poses[now_index].pose.position.y,
				current_time.toSec());
		now_x_velocity = interpolate(velocities_.poses[now_index - 1].header.stamp.toSec(),
				velocities_.poses[now_index].header.stamp.toSec(),
				velocities_.poses[now_index - 1].pose.position.x,
				velocities_.poses[now_index].pose.position.x,
				current_time.toSec());
		now_y_velocity = interpolate(velocities_.poses[now_index - 1].header.stamp.toSec(),
				velocities_.poses[now_index].header.stamp.toSec(),
				velocities_.poses[now_index - 1].pose.position.y,
				velocities_.poses[now_index].pose.position.y,
				current_time.toSec());	

		now_orientation = interpolate(path_.poses[now_index - 1].header.stamp.toSec(),
				path_.poses[now_index].header.stamp.toSec(),
				getYaw(path_.poses[now_index - 1].pose.orientation),
				getYaw(path_.poses[now_index].pose.orientation),
				current_time.toSec());

		now_velocities_orientation = interpolate(velocities_.poses[now_index - 1].header.stamp.toSec(),
				velocities_.poses[now_index].header.stamp.toSec(),
				getYaw(velocities_.poses[now_index - 1].pose.orientation),
				getYaw(velocities_.poses[now_index].pose.orientation),
				current_time.toSec());
	}
	else
	{
		now_x = path_.poses[0].pose.position.x;
		now_y = path_.poses[0].pose.position.y;
		now_x_velocity = velocities_.poses[0].pose.position.x;
		now_y_velocity = velocities_.poses[0].pose.position.y;
		now_orientation = getYaw(path_.poses[0].pose.orientation);
		now_velocities_orientation = getYaw(velocities_.poses[0].pose.orientation);
	}

	ROS_INFO_STREAM("now coordinates: " << now_index << " (" << now_x << ", " << now_y << ", " << now_orientation << ")");
	ROS_INFO_STREAM("now velocity: " << now_index << " (" << now_x_velocity << ", " << now_y_velocity << ", " << now_velocities_orientation << ")");
#endif
	// Convert back to quaternion
	tf2::Quaternion q_final_tf = tf2::Quaternion(tf2Scalar(0), tf2Scalar(0), tf2Scalar(final_orientation));
	geometry_msgs::Quaternion q_final = tf2::toMsg(q_final_tf);

	tf2::Quaternion q_final_tf_velocity = tf2::Quaternion(tf2Scalar(0), tf2Scalar(0), tf2Scalar(final_velocities_orientation));
	geometry_msgs::Quaternion q_final_velocity = tf2::toMsg(q_final_tf_velocity);

	// Return Pose of target position
	geometry_msgs::Pose target_pos;
	target_pos.position.x = final_x;
	target_pos.position.y = final_y;
	target_pos.position.z = 0;
	target_pos.orientation = q_final;

	geometry_msgs::Vector3 target_velocity;
	target_velocity.x = final_x_velocity;
	target_velocity.y = final_y_velocity;
	target_velocity.z = 0;
	// TODO incorporate angular velocity somehow

	PositionVelocity target;
	target.position = target_pos;
	target.velocity = target_velocity;

	// Should actually return total distance now
	total_distance_travelled = vec_path_length_[now_index - 1];
	current_index = now_index - 1;
#ifdef DEBUG
	ROS_INFO_STREAM("Successfully returned target position and orientation");
#endif
	return target;
}
