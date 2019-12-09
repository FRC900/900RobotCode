// Class to hold current alignment state of a particular axis
// Holds various pub and sub topics to use as well as error and timeout status
#pragma once

#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float64MultiArray.h"

class AlignActionAxisState
{
	public:
		AlignActionAxisState(const std::string &name,
					  ros::NodeHandle &nh,
				const std::string &enable_pub_topic,
				const std::string &error_sub_topic,
				const boost::function<void(const std_msgs::Float64MultiArrayConstPtr &msg)> &error_sub_cb,
				const double timeout,
				const double error_threshold)
			: enable_pub_(nh.advertise<std_msgs::Bool>(enable_pub_topic, 1, true))
			, error_sub_(nh.subscribe(error_sub_topic, 1, error_sub_cb))
			, aligned_(false)
			, error_(0.0)
			, timeout_(timeout)
			, error_threshold_(error_threshold)
			, timed_out_(false)
		{
		}
		ros::Publisher enable_pub_;
		ros::Subscriber error_sub_;
		bool aligned_;
		double error_;
		double timeout_;
		double error_threshold_;
		bool timed_out_;
};

