#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>

geometry_msgs::Twist cmd_vel_msg;
ros::Time time_since_command;
ros::Time current_time;

void orientCB(const std_msgs::Float64& msg)
{
	time_since_command = ros::Time::now();
	double command = msg.data;
	cmd_vel_msg.angular.z = command;
}

int main(int argc, char ** argv)
{
	ros::init(argc, argv, "publish_pid_cmd_vel");
	ros::NodeHandle nh;

	ros::Subscriber orient_pid_sub = nh.subscribe("orient_pid/orient_control_effort", 1, &orientCB);

	ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("swerve_drive_controller/cmd_vel", 1);

	while(ros::ok())
	{
		current_time = ros::Time::now();
		if((current_time - time_since_command).toSec() < 1 /*&& (current_time - time_since_y).toSec() < 1*/)
		{
			cmd_vel_pub.publish(cmd_vel_msg);
		}
		ros::spinOnce();
	}
	return 0;
}
