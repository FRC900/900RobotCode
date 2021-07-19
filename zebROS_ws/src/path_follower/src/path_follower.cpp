/* 1. information you have: current position and velcocity of the robot, waypoints already limited for velocity and acceleration
 * 2. find path point closest to the position of the robot
 * 3. find the goal point -- that is, point which the robot should drive to (based on time_offset setting to grab a target point in the future)
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
	}
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
geometry_msgs::Pose PathFollower::run(double &total_distance_travelled)
{
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

	size_t current_waypoint_index = 0; //the index BEFORE the point on the path
	// Timestamps in path are relative to when they were created.
	// Offset them here to be relative the time the path starts running
	const ros::Time current_time = ros::Time::now() - start_time_offset_ + ros::Duration(path_.poses[0].header.stamp.toSec());

	const size_t last_index = num_waypoints_ - 1;
	double final_x, final_y, final_orientation;

	// Find point in path closest to odometry reading
	ROS_INFO_STREAM("num_waypoints = " << num_waypoints_);


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
	final_orientation = interpolate(path_.poses[index - 1].header.stamp.toSec(),
			path_.poses[index].header.stamp.toSec(),
			getYaw(path_.poses[index - 1].pose.orientation),
			getYaw(path_.poses[index].pose.orientation),
			current_time.toSec());

	ROS_INFO_STREAM("drive to coordinates: " << index << " (" << final_x << ", " << final_y << ", " << final_orientation << ")");

	// This is strictly for the debugging ROS_INFO below
	double now_x;
	double now_y;
	double now_orientation;
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
		now_orientation = interpolate(path_.poses[now_index - 1].header.stamp.toSec(),
				path_.poses[now_index].header.stamp.toSec(),
				getYaw(path_.poses[now_index - 1].pose.orientation),
				getYaw(path_.poses[now_index].pose.orientation),
				current_time.toSec());
	}
	else
	{
		now_x = path_.poses[0].pose.position.x;
		now_y = path_.poses[0].pose.position.y;
		now_orientation = getYaw(path_.poses[0].pose.orientation);
	}

	ROS_INFO_STREAM("now coordinates: " << now_index << " (" << now_x << ", " << now_y << ", " << now_orientation << ")");

	// Convert back to quaternion
	tf2::Quaternion q_final_tf = tf2::Quaternion(tf2Scalar(0), tf2Scalar(0), tf2Scalar(final_orientation));
	geometry_msgs::Quaternion q_final = tf2::toMsg(q_final_tf);

	// Return Pose of target position
	geometry_msgs::Pose target_pos;
	target_pos.position.x = final_x;
	target_pos.position.y = final_y;
	target_pos.position.z = 0;
	target_pos.orientation = q_final;

	total_distance_travelled = vec_path_length_[current_waypoint_index];

	ROS_INFO_STREAM("Successfully returned target position and orientation");
	return target_pos;
}
