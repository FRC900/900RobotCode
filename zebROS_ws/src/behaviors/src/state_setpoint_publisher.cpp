#include <ros/ros.h>

/*
 * Publishes orient state, orient setpoint, y state, y setpoint, and pid_enable
 */

#include <screen_to_world/WorldVector.h>
#include "std_msgs/Float64.h"
#include "teraranger_array/RangeArray.h"
#include "std_msgs/Bool.h"
#include "std_srvs/SetBool.h"
#include "sensor_msgs/JointState.h"

bool publish;

double navx_angle_state;
double distance_state;

std_msgs::Float64 orient_state_msg;
std_msgs::Float64 y_state_msg;
std_msgs::Float64 orient_setpoint_msg;
std_msgs::Float64 y_target_msg;
std_msgs::Bool pid_enable_msg;

ros::Subscriber world_vector_sub;
ros::Publisher orient_setpoint;
ros::Publisher orient_state;
ros::Publisher y_setpoint;
ros::Publisher y_state;
ros::Publisher navx_setpoint;
ros::Publisher navx_state;
ros::Publisher pid_enable;
ros::ServiceServer run_align;

double distance_between_sensors = 0.3937;

void distanceCB(const teraranger_array::RangeArray& msg)
{
	distance_state = atan((msg.ranges[1].range - msg.ranges[5].range) / distance_between_sensors);
	ROS_INFO_STREAM("distance state = " << distance_state);
}

void jointStatesCB(const sensor_msgs::JointState& msg)
{
	navx_angle_state = msg.position[2];
	ROS_INFO_STREAM("navx state = " << navx_angle_state);
}

bool align_service(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
	publish = req.data;
	res.success = true;
	return 0;
}

int main(int argc, char ** argv)
{
	ros::init(argc, argv, "distance_setpoint_pub");
	ros::NodeHandle nh;
	int sensor_used = 1; //0 for navx, 1 for distance sensors, sorry!!

	//the orientation we want is 0
	orient_setpoint_msg.data = 0;

	//state subscribers
	ros::Subscriber distance_sub = nh.subscribe("multiflex_1/ranges_raw", 1, distanceCB);
	ros::Subscriber angle_sub = nh.subscribe("joint_states", 1, jointStatesCB);
	
	//required pid_enable topic for the pid node
	pid_enable = nh.advertise<std_msgs::Bool>("pid_enable", 1);

	//sets true/false to PID node
	run_align = nh.advertiseService("run_align", align_service);

	//setpoint and state publishers
	orient_setpoint = nh.advertise<std_msgs::Float64>("orient_setpoint", 1);
	orient_state = nh.advertise<std_msgs::Float64>("orient_state", 1);
	y_setpoint = nh.advertise<std_msgs::Float64>("y_setpoint", 1);
	y_state = nh.advertise<std_msgs::Float64>("y_state", 1);
	navx_setpoint = nh.advertise<std_msgs::Float64>("navx_setpoint", 1);
	navx_state = nh.advertise<std_msgs::Float64>("navx_state", 1);

	while(ros::ok())
	{
		orient_setpoint.publish(orient_setpoint_msg);

		switch (sensor_used)
		{
			case 0:
				orient_state_msg.data = navx_angle_state;
				break;
			case 1:
				orient_state_msg.data = distance_state;
				break;
			default:
				orient_state_msg.data = 0;
				break;
		}

		orient_state.publish(orient_state_msg);

		pid_enable_msg.data = publish;
		pid_enable.publish(pid_enable_msg);

		ros::spinOnce();
	}

	return 0;
}
