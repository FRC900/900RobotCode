#include "dynamic_reconfigure_wrapper/dynamic_reconfigure_wrapper.h"
#include "teleop_joystick_control/rate_limiter.h"
#include "teleop_joystick_control/TeleopJoystickCompConfig.h"

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>

bool enable_combination = false;

geometry_msgs::Twist combined_cmd_vel;

ros::Publisher combined_cmd_vel_pub;

teleop_joystick_control::TeleopJoystickCompConfig config;

std::unique_ptr<rate_limiter::RateLimiter> rotation_rate_limiter;

void publishCombinedCmdVel(void)
{
	if(enable_combination)
		combined_cmd_vel_pub.publish(combined_cmd_vel);
}

void enableCallback(const std_msgs::Bool::ConstPtr &enable_msg)
{
	enable_combination = enable_msg->data;
}

void teleopCallback(const geometry_msgs::Twist::ConstPtr &teleop_msg)
{
	combined_cmd_vel.linear.x = teleop_msg->linear.x;
	combined_cmd_vel.linear.y = teleop_msg->linear.y;
}

void orientCallback(const std_msgs::Float64::ConstPtr &orient_msg)
{
	const double rotation = rotation_rate_limiter->applyLimit(orient_msg->data, ros::Time::now());
	combined_cmd_vel.angular.z = -rotation;
}

int main(int argc, char ** argv)
{
	ros::init(argc, argv, "combine_orient_strafing_node");
	ros::NodeHandle n;
	ros::NodeHandle n_params(n, "teleop_params");

	if(!n_params.getParam("max_rot", config.max_rot))
	{
		ROS_ERROR("Could not read max_rot in orient_strafing");
	}
	if(!n_params.getParam("rotate_rate_limit_time", config.rotate_rate_limit_time))
	{
		ROS_ERROR("Could not read rotate_rate_limit_time in orient_strafing");
	}

	DynamicReconfigureWrapper<teleop_joystick_control::TeleopJoystickCompConfig> drw(n_params, config);

	rotation_rate_limiter = std::make_unique<rate_limiter::RateLimiter>(-config.max_rot, config.max_rot, config.drive_rate_limit_time);

	combined_cmd_vel.linear.x = 0.0;
	combined_cmd_vel.linear.y = 0.0;
	combined_cmd_vel.linear.z = 0.0;
	combined_cmd_vel.angular.x = 0.0;
	combined_cmd_vel.angular.y = 0.0;
	combined_cmd_vel.angular.z = 0.0;

	ros::Subscriber enable_sub = n.subscribe("pid_enable", 5, enableCallback);
	ros::Subscriber teleop_sub = n.subscribe("/teleop/swerve_drive_controller/cmd_vel", 5, teleopCallback);
	ros::Subscriber orient_sub = n.subscribe("control_effort", 5, orientCallback);

	combined_cmd_vel_pub = n.advertise<geometry_msgs::Twist>("swerve_drive_controller/cmd_vel", 5);

	ros::Rate r = 100;

	while(ros::ok())
	{
		publishCombinedCmdVel();
		ros::spinOnce();
		r.sleep();
	}

	return 0;
}
