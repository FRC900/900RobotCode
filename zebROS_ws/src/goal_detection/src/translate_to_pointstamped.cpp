#include "ros/ros.h"
#include "geometry_msgs/PointStamped.h"
#include "field_obj/Detection.h"

ros::Publisher pub;

void callback(const field_obj::Detection &msg)
{
	size_t num_goals = msg.objects.size();
	int index;
	if (num_goals == 0)
	{
		ROS_INFO_STREAM_THROTTLE(0.25, "No goals found. Skipping");
		return;
	}
	else if (num_goals > 1)
	{
		double min_distance = std::numeric_limits<double>::max();
		index = -1;
		for(size_t i = 0; i < num_goals; i++)
		{
			if(msg.objects[i].location.x < min_distance)
			{
				min_distance = msg.objects[i].location.x;
				index = i;
			}
		}
		if(index == -1)
		{
			ROS_INFO_STREAM_THROTTLE(0.25, "No goals found that are not infinitely far away. Skipping.");
			return;
		}
	}
	else
	{
		index = 0;
	}
	geometry_msgs::PointStamped goal_location;
	goal_location.header = msg.header;
	goal_location.point.x = msg.objects[index].location.x;
	goal_location.point.y = msg.objects[index].location.y;
	goal_location.point.z = 0;
	pub.publish(goal_location);
}

int main(int argc, char ** argv)
{
	ros::init(argc, argv, "translate_to_pointstamped");
	ros::NodeHandle nh;

	ros::Subscriber sub = nh.subscribe("goal_detect_msg", 2, &callback);
	pub = nh.advertise<geometry_msgs::PointStamped>("pointstamped_goal_msg", 1);

	ros::spin();

	return 0;
}
