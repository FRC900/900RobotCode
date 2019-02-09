#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>

geometry_msgs::Twist cmd_vel_msg;
ros::Time time_since_command;
ros::Time current_time;

std::string orient_topic;
std::string x_topic;
std::string y_topic;

void orientCB(const std_msgs::Float64& msg)
{
	time_since_command = ros::Time::now();
	cmd_vel_msg.angular.z = msg.data;
}
void xCB(const std_msgs::Float64& msg)
{
	time_since_command = ros::Time::now();
	cmd_vel_msg.linear.x = msg.data;
}
void yCB(const std_msgs::Float64& msg)
{
	time_since_command = ros::Time::now();
	cmd_vel_msg.linear.y = msg.data;
}

int main(int argc, char ** argv)
{
	ros::init(argc, argv, "publish_pid_cmd_vel");
	ros::NodeHandle nh;
    ros::NodeHandle nh_private_params("~");

    if(!nh_private_params.getParam("orient_topic", orient_topic)) {
        ROS_ERROR("Could not read orient_topic in publish_pid_cmd_vel");
    }
    else {
        ros::Subscriber orient_pid_sub = nh.subscribe(orient_topic, 1, &orientCB);
    }
    if(!nh_private_params.getParam("x_topic", x_topic)) {
        ROS_ERROR("Could not read x_topic in publish_pid_cmd_vel");
    }
    else {
        ros::Subscriber x_pid_sub = nh.subscribe(x_topic, 1, &xCB);
    }
    if(!nh_private_params.getParam("y_topic", y_topic)) {
        ROS_ERROR("Could not read y_topic in publish_pid_cmd_vel");
    }
    else {
        ros::Subscriber y_pid_sub = nh.subscribe(y_topic, 1, &yCB);
    }


	ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("swerve_drive_controller/cmd_vel", 1);

	ros::Rate r(100);

	while(ros::ok())
	{
		current_time = ros::Time::now();
		if((current_time - time_since_command).toSec() < 1 /*&& (current_time - time_since_y).toSec() < 1*/)
		{
			cmd_vel_pub.publish(cmd_vel_msg);
		}
		ros::spinOnce();
		r.sleep();
	}
	return 0;
}
