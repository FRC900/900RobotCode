#include "ros/ros.h"
#include "color_spin/color_algorithm.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "color_algorithm");


	ros::NodeHandle nh;
	ros::ServiceClient client = nh.serviceClient<color_spin::color_algorithm>("color_algorithm");
	color_spin::color_algorithm srv;
	srv.request.fms_color = atoll(argv[1]);
	srv.request.sensor_color = atoll(argv[2]);
	if (client.call(srv))
	{
		ROS_INFO("rotation: %1f", (float)srv.response.rotate);
	}
	else
	{
		ROS_ERROR("Failed to call service oh no color_algorithm");
		return 1;
	}

	return 0;

}

