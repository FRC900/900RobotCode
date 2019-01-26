#include <ros/ros.h>

/*
 * Publishes orient state, orient setpoint, y state, y setpoint, and pid_enable
 */

#include <screen_to_world/WorldVector.h>
#include "std_msgs/Float64.h"
#include "teraranger_array/RangeArray.h"
#include "std_msgs/Bool.h"
#include "std_srvs/SetBool.h"

bool publish;

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
ros::Publisher pid_enable;
ros::ServiceServer run_align;

double distance_between_sensors = 0.3937;

void distanceCB(const teraranger_array::RangeArray& msg)
{
	orient_setpoint.publish(orient_setpoint_msg);

	double angle = atan((msg.ranges[1].range - msg.ranges[5].range) / distance_between_sensors);
	orient_state_msg.data = angle;
	orient_state.publish(orient_state_msg);

	pid_enable_msg.data = publish;
	pid_enable.publish(pid_enable_msg);
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

	orient_setpoint_msg.data = 0;

	ros::Subscriber distance_sub = nh.subscribe("multiflex_1/ranges_raw", 1, distanceCB);
	orient_setpoint = nh.advertise<std_msgs::Float64>("orient_setpoint", 1);
	orient_state = nh.advertise<std_msgs::Float64>("orient_state", 1);
	//y_setpoint = nh.advertise<std_msgs::Float64>("y_setpoint", 1);
	//y_state = nh.advertise<std_msgs::Float64>("y_state", 1);
	pid_enable = nh.advertise<std_msgs::Bool>("pid_enable", 1);
	run_align = nh.advertiseService("run_align", align_service);

	ros::spin();

	return 0;
}
