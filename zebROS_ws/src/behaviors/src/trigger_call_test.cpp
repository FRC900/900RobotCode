#include "ros/ros.h"
#include "std_srvs/Trigger.h"

int main(int argc, char **argv)
{
	ros::init(argc,argv,"trigger_call_test");
	ros::NodeHandle nh;
	ros::ServiceClient cli = nh.serviceClient<std_srvs::Trigger>("frcrobot_jetson/align_service");
	std_srvs::Trigger trg;
	ROS_INFO("calling server...");
	if(cli.call(trg))
	{
		ROS_INFO("server called");
	} else {
		ROS_INFO("failed");
	}
}
