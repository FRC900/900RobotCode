#include <ros/ros.h>
#include <vector>
#include "teraranger_array/RangeArray.h"
#include "geometry_msgs/Twist.h"

#define NUM_SENSORS 4
#define MIN_DIST 4

std::vector<bool> senses_panels;

void multiflexCB(const teraranger_array::RangeArray& msg)
{
	for(int i = 0; i < msg.ranges.size(); i++)
	{
		ROS_INFO_STREAM("i = " << i << " range = " << msg.ranges[i].range);
		if(msg.ranges[i].range == msg.ranges[i].range)
			senses_panels[i] = (msg.ranges[i].range < 1.0);
		else
			senses_panels[i] = false;
	}
}

int main(int argc, char ** argv)
{
	ros::init(argc, argv, "align_with_terabee");
	ros::NodeHandle n;

	senses_panels.resize(NUM_SENSORS);

	int location;

	ros::Subscriber terabee_sub = n.subscribe("multiflex_1/ranges_raw", 1, &multiflexCB);
	ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("swerve_drive_controller/cmd_vel", 1);

	geometry_msgs::Twist cmd_vel_msg;
	cmd_vel_msg.linear.x = 0;
	cmd_vel_msg.linear.y = 0;
	cmd_vel_msg.linear.z = 0;
	cmd_vel_msg.angular.x = 0;
	cmd_vel_msg.angular.y = 0;
	cmd_vel_msg.angular.z = 0;

	ros::Rate r(10);

	while(ros::ok())
	{
		ROS_INFO_STREAM("0: " << senses_panels[0] << "1: " << senses_panels[1] << "2: " << senses_panels[2] << "3: " << senses_panels[3]); 
		ROS_INFO_STREAM("0: " << senses_panels[0] << "1: " << senses_panels[1] << "2: " << senses_panels[2] << "3: " << senses_panels[3]); 
		bool cutout_found = false;
		for(int i = 0; i < NUM_SENSORS - 1; i++)
		{
			if(!senses_panels[i] && !senses_panels[i+1])
			{
				location = i;
				if(cutout_found)
					ROS_ERROR_STREAM_THROTTLE(1, "two cutouts found????");
				cutout_found = true;
			}
		}
		ROS_INFO_STREAM("location = " << location);

		if(location == floor((NUM_SENSORS-1)/2))
		{
			ROS_INFO_STREAM("PERFECT");
			cmd_vel_msg.linear.y = 0;
		}
		else if(location < floor((NUM_SENSORS-1)/2))
		{
			ROS_INFO_STREAM("move left");
			cmd_vel_msg.linear.y = 0.1;
		}
		else
		{
			ROS_INFO_STREAM("move right");
			cmd_vel_msg.linear.y = -0.1;
		}

		cmd_vel_pub.publish(cmd_vel_msg);

		ros::spinOnce();
		r.sleep();
	}

	return 0;
}
