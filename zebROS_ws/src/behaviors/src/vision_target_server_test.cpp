#include "ros/ros.h"
#include "geometry_msgs/Point32.h"

int main(int argc, char **argv)
{
	ros::init(argc,argv, "vision_target_server_test");
	ros::NodeHandle n;
	ros::Publisher pub = n.advertise<geometry_msgs::Point32>("vision_target",1);
	ros::Rate rate(10);
	while (ros::ok())
	{
		geometry_msgs::Point32 point;
		point.x = 10.0;
		point.y = 10.0;
		point.z = 0.5;
		pub.publish(point);
		ros::spinOnce();
		rate.sleep();
	}
	return 0;
}
