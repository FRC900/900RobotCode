#pragma once

#include <ros/ros.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Point32.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <string>
#include <control_toolbox/pid.h>
//#include <tf/transform_datatypes.h>
//#include <tf/tf.h>
#include <geometry_msgs/Pose.h>
#include <math.h>

namespace tf2
{
template <>
inline
const ros::Time &getTimestamp(const nav_msgs::Odometry &t)
{
	return t.header.stamp;
}

template <>
inline
const std::string &getFrameId(const nav_msgs::Odometry &t)
{
	return t.header.frame_id;
}

template <>
inline
void doTransform(const geometry_msgs::TwistWithCovarianceStamped &t_in, geometry_msgs::TwistWithCovarianceStamped &t_out, const geometry_msgs::TransformStamped &transform)
{
	geometry_msgs::Vector3 vector_in;
	vector_in = t_in.twist.twist.linear;
	geometry_msgs::Vector3 linear_vector_out;
	doTransform(vector_in, linear_vector_out, transform);

	vector_in = t_in.twist.twist.angular;
	geometry_msgs::Vector3 angular_vector_out;
	doTransform(vector_in, angular_vector_out, transform);

	geometry_msgs::PoseWithCovariance::_covariance_type covariance_in;
	covariance_in = t_in.twist.covariance;
	geometry_msgs::PoseWithCovariance::_covariance_type covariance_out;
	tf2::Transform tf2_transform;
	fromMsg(transform.transform, tf2_transform);
	covariance_out = transformCovariance(covariance_in, tf2_transform);

	t_out.header = t_in.header;
	t_out.twist.twist.linear = linear_vector_out;
	t_out.twist.twist.angular = angular_vector_out;
	t_out.twist.covariance = covariance_out;
}

template <>
inline
void doTransform(const nav_msgs::Odometry &t_in, nav_msgs::Odometry &t_out, const geometry_msgs::TransformStamped &transform)
{
	geometry_msgs::PoseWithCovarianceStamped pose_in;
	pose_in.header = t_in.header;
	pose_in.pose = t_in.pose;
	geometry_msgs::PoseWithCovarianceStamped pose_out;
	doTransform(pose_in, pose_out, transform);

	geometry_msgs::TwistWithCovarianceStamped twist_in;
	twist_in.header = t_in.header;
	twist_in.twist = t_in.twist;
	geometry_msgs::TwistWithCovarianceStamped twist_out;
	doTransform(twist_in, twist_out, transform);

	t_out.header = t_in.header;
	t_out.pose = pose_out.pose;
	t_out.twist = twist_out.twist;
}
}


class PathFollower
{
	private:
		nav_msgs::Path path_;

		double lookahead_distance_;
		size_t num_waypoints_;
		double path_length_;
		std::vector<double> vec_path_length_;
		double start_point_radius_;

	public:
		PathFollower(double lookahead_distance,
					 double start_point_radius)
		{
			lookahead_distance_ = lookahead_distance;
			start_point_radius_ = start_point_radius;
		}

		// load nav_msgs::Path
		bool loadPath(const nav_msgs::Path &path);

		// get yaw from geometry_msgs quaternion
		static double getYaw(const geometry_msgs::Quaternion q);

		// path length getter
		const double getPathLength();

		// interpolation function, just for cleanness 
		double interpolate(double start_t, double end_t, double start_x, double end_x, double current_t) const;

		// contains the main control loop
		geometry_msgs::Pose run(nav_msgs::Odometry odom, double &distance_travelled);
};

