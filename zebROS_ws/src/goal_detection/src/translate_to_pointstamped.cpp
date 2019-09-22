#include "ros/ros.h"
#include "geometry_msgs/PointStamped.h"
#include "goal_detection/GoalDetection.h"

ros::Publisher zed_point_msg_pub;
ros::Publisher c920_point_msg_pub;

void ZEDMsgCallback(const goal_detection::GoalDetection &msg)
{
	size_t num_goals = msg.location.size();
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
			if(msg.location[i].x < min_distance)
			{
				min_distance = msg.location[i].x;
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
	goal_location.point.x = msg.location[index].y; //Inverted on purpose
	goal_location.point.y = msg.location[index].x;
	goal_location.point.z = 0;
	zed_point_msg_pub.publish(goal_location);
}
void C920MsgCallback(const goal_detection::GoalDetection &msg)
{
	size_t num_goals = msg.location.size();
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
			if(msg.location[i].x < min_distance)
			{
				min_distance = msg.location[i].x;
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
	goal_location.point.x = msg.location[index].x;
	goal_location.point.y = msg.location[index].y;
	goal_location.point.z = 0;
	c920_point_msg_pub.publish(goal_location);
}

int main(int argc, char ** argv)
{
	ros::init(argc, argv, "translate_to_pointstamped");
	ros::NodeHandle nh;

	ros::Subscriber zed_msg_sub = nh.subscribe("/goal_detection/goal_detect_msg", 2, &ZEDMsgCallback);
	ros::Subscriber c920_msg_sub = nh.subscribe("/c920_goal_detection/goal_detect_msg", 2, &C920MsgCallback);
	zed_point_msg_pub = nh.advertise<geometry_msgs::PointStamped>("/align_hatch/pointstamped_goal_msg", 1);
	c920_point_msg_pub = nh.advertise<geometry_msgs::PointStamped>("/align_cargo/pointstamped_goal_msg", 1);

	ros::spin();

	return 0;
}
