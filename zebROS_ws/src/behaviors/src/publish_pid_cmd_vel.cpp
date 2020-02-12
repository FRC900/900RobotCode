#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include <atomic>
#include <cmath>

ros::Time time_since_command;
ros::Time time_since_orient;
ros::Time time_since_x;
ros::Time time_since_y;
ros::Time time_since_pid_enable;
ros::Time time_at_last_orient_state;
ros::Time current_time;

std::string orient_topic;
std::string x_topic;
std::string y_topic;
std::string enable_topic;
std::string ratio_xy_topic;
std::string name;
std::string orient_state_topic;

double command_timeout = 0.5; //Default value if param not loaded

ros::Subscriber x_pid_sub;
ros::Subscriber y_pid_sub;
ros::Subscriber orient_pid_sub;
ros::Subscriber enable_pid_sub;
ros::Subscriber ratio_xy_sub;
ros::Subscriber orient_state_sub;

geometry_msgs::Twist cmd_vel_msg;

std::atomic<double> ratio_xy(0.0);
bool pid_enable = false;
bool orient_sub = false;
bool x_sub = false;
bool y_sub = false;
bool ratio_imposed = false;
double current_angle = 0;
double x_command = 0;
double y_command = 0;

void orientCB(const std_msgs::Float64& msg)
{
	time_since_command = ros::Time::now();
	time_since_orient = ros::Time::now();
	cmd_vel_msg.angular.z = -1*((msg.data == msg.data) ? msg.data : 0.0);
}
void xCB(const std_msgs::Float64& msg)
{
	time_since_command = ros::Time::now();
	time_since_x = ros::Time::now();
	x_command = ((msg.data == msg.data) ? msg.data : 0.0);
}
void yCB(const std_msgs::Float64& msg)
{
	time_since_command = ros::Time::now();
	time_since_y = ros::Time::now();
	y_command = ((msg.data == msg.data) ? msg.data : 0.0);
}
void enableCB(const std_msgs::Bool& msg)
{
	time_since_pid_enable = ros::Time::now();
	pid_enable = msg.data;
}
void ratio_xyCB(const std_msgs::Float64& msg) {
	ratio_xy = msg.data;
}
void orientStateCB(const std_msgs::Float64& msg) {
        time_at_last_orient_state = ros::Time::now();
		if(msg.data == msg.data)
			current_angle = msg.data;
		else
			current_angle = 0;
}

int main(int argc, char ** argv)
{
	ros::init(argc, argv, "publish_pid_cmd_vel");
	ros::NodeHandle nh;
    ros::NodeHandle nh_private_params("~");

    if(!nh_private_params.getParam("orient_topic", orient_topic)) {
        ROS_INFO("Could not read orient_topic in publish_pid_cmd_vel");
    }
    else {
		ROS_WARN_STREAM("Subscribing to: " << orient_topic);
        orient_pid_sub = nh.subscribe(orient_topic, 1, &orientCB);
		orient_sub = true;
    }
    if(!nh_private_params.getParam("x_topic", x_topic)) {
        ROS_ERROR("Could not read x topic in publish_pid_cmd_vel");
    }
    else {
		ROS_WARN_STREAM("Subscribing to: " << x_topic);
        x_pid_sub = nh.subscribe(x_topic, 1, &xCB);
		x_sub = true;
    }
    if(!nh_private_params.getParam("y_topic", y_topic)) {
        ROS_INFO("Could not read y_topic in publish_pid_cmd_vel");
    }
    else {
		ROS_WARN_STREAM("Subscribing to: " << y_topic);
        y_pid_sub = nh.subscribe(y_topic, 1, &yCB);
		y_sub = true;
    }
	if(!nh_private_params.getParam("enable_topic", enable_topic))
	{
		ROS_ERROR("Could not read enable_topic in publish_pid_cmd_vel");
	}
	else {
		enable_pid_sub = nh.subscribe(enable_topic, 1, &enableCB);
	}
	if(!nh_private_params.getParam("command_timeout", command_timeout))
	{
		ROS_ERROR("Could not read command_timeout in publish_pid_cmd_vel");
	}
	if(!nh_private_params.getParam("ratio_xy_topic", ratio_xy_topic))
	{
		ROS_ERROR("Could not read ratio_xy_topic in publish_pid_cmd_vel");
		ratio_imposed = false;
	}
	else {
        ratio_xy_sub = nh.subscribe(ratio_xy_topic, 1, &ratio_xyCB);
		ratio_imposed = true;
	}
	if(!nh_private_params.getParam("name", name))
	{
		ROS_ERROR("Could not read name in publish_pid_cmd_vel");
	}
        if(!nh_private_params.getParam("orient_state_topic", orient_state_topic))
        {
            ROS_ERROR("Could not read orient_state_topic in publish_pid_cmd_vel. Assuming robot_centric.");
        }
        else {
            orient_state_sub = nh.subscribe(orient_state_topic, 1, &orientStateCB);
        }

	ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>(name + "/swerve_drive_controller/cmd_vel", 1);

	ros::Rate r(100);

	time_since_pid_enable = ros::Time::now();
	while(ros::ok())
	{
		current_time = ros::Time::now();
		if((current_time - time_since_command).toSec() < command_timeout && pid_enable)
		{
			if((current_time - time_since_orient).toSec() > 0.1)
				cmd_vel_msg.angular.z = 0.0;
			if((current_time - time_since_x).toSec() > 0.1)
				cmd_vel_msg.linear.x = 0.0;
			if((current_time - time_since_y).toSec() > 0.1)
				cmd_vel_msg.linear.y = 0.0;
			if(ratio_imposed) {
				if(x_sub && y_sub) {
					ROS_ERROR("publish pid cmd_vel: Can't subscribe to both x and y and impose a ratio");
				}
				else {
					if(x_sub) {
						if(fabs(ratio_xy) > 1e5) {
							cmd_vel_msg.linear.y = cmd_vel_msg.linear.x / ratio_xy;
						}
						else {
							cmd_vel_msg.linear.y = 0;
						}
					}
					else {
						cmd_vel_msg.linear.x = cmd_vel_msg.linear.y * ratio_xy;
					}
				}
			}
			time_since_x = ros::Time::now();
                        double rotate_angle;
                        if((current_time - time_at_last_orient_state).toSec() < command_timeout)
                        {
                            rotate_angle = -1 * current_angle;
                        }
                        else
                        {
                            rotate_angle = 0;
                        }
                        cmd_vel_msg.linear.x = x_command * cos(rotate_angle) - y_command * sin(rotate_angle);
                        cmd_vel_msg.linear.y = x_command * sin(rotate_angle) + y_command * cos(rotate_angle);
			cmd_vel_pub.publish(cmd_vel_msg);
		}
		else {
			cmd_vel_msg.angular.z = 0.0;
			cmd_vel_msg.linear.x = 0.0;
			cmd_vel_msg.linear.y = 0.0;
			cmd_vel_pub.publish(cmd_vel_msg);
		}
		ros::spinOnce();
		r.sleep();
	}
	return 0;
}
