#include <ros/ros.h>
#include <vector>
#include "teraranger_array/RangeArray.h"
#include "geometry_msgs/Twist.h"

#define NUM_SENSORS 4
#define MIN_DIST 4

std::vector<bool> senses_panels;

// I get what you're going for here - either the reading is "in" or "out" of the key
// You might get better results by using the closest sensor reading as a baseline
// for "out". Then other readings are set based on whether they are within a range
// of that closest distance.  You'd still want a max range check, though, to avoid counting
// really wacky results. It might also be useful for setting a range that's valid
// for "in" values. Anything further away than that might be from e.g. the sensor pointing
// at nothing?  Could be useful for detecting if e.g. only one sensor is seeing anything
// So you'd end up with 3 possible states :
//  seeing the normal plane of the cargo ship
//  seeing inside the rectangle cutout
//  seeing something else
//
// Making a table with the 4 sensors seeing permutations of each of the 3 states
// would give ideas about where to move the robot?
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

	// TODO : I'd increase the rate 
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

		// TODO : calcs using only ints will result in an int,
		// no need for floor here.
		if(location == floor((NUM_SENSORS-1)/2))
		{
			ROS_INFO_STREAM("PERFECT");
			cmd_vel_msg.linear.y = 0;
		}
		else if(location < floor((NUM_SENSORS-1)/2))
		{
			ROS_INFO_STREAM("move left");
			cmd_vel_msg.linear.y = 0.1; // TODO : this might not be enough to get the robot to move?
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
