#include <ros/ros.h>
#include <vector>
#include "teraranger_array/RangeArray.h"
#include "geometry_msgs/Twist.h"

#define NUM_SENSORS 4
#define MIN_DIST 4

std::vector<bool> senses_panels;
std::vector<double> sensors_distances;
std::vector<int> ternary_distances;
double min_dist = 100000000000;

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
		if(msg.ranges[i].range == msg.ranges[i].range)
		{
			sensors_distances[i] = msg.ranges[i].range;
			min_dist = std::min(min_dist, static_cast<double>(msg.ranges[i].range));
			ROS_INFO_STREAM("i = " << i << " range = " << sensors_distances[i]);
			senses_panels[i] = (msg.ranges[i].range < 1.0);
		}
		else
		{
			senses_panels[i] = false;
			sensors_distances[i] = 10000000000000000000;
		}
	}
}

int main(int argc, char ** argv)
{
	ros::init(argc, argv, "align_with_terabee");
	ros::NodeHandle n;

	senses_panels.resize(NUM_SENSORS);
	sensors_distances.resize(NUM_SENSORS);
	ternary_distances.resize(NUM_SENSORS);

	int location;
	double dist_to_back_panel = 0.07;
	double error_threshhold = 0.03;

	ros::Subscriber terabee_sub = n.subscribe("multiflex_1/ranges_raw", 1, &multiflexCB);
	ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("swerve_drive_controller/cmd_vel", 1);

	geometry_msgs::Twist cmd_vel_msg;
	cmd_vel_msg.linear.x = 0;
	cmd_vel_msg.linear.y = 0;
	cmd_vel_msg.linear.z = 0;
	cmd_vel_msg.angular.x = 0;
	cmd_vel_msg.angular.y = 0;
	cmd_vel_msg.angular.z = 0;

	ros::Rate r(50);

	while(ros::ok())
	{
		ROS_INFO_STREAM("0: " << senses_panels[0] << "1: " << senses_panels[1] << "2: " << senses_panels[2] << "3: " << senses_panels[3]); 

		for(int i = 0; i < sensors_distances.size(); i++)
		{
			if(sensors_distances[i] != sensors_distances[i])
				ternary_distances[i] = 2;
			else if(fabs(sensors_distances[i] - min_dist) < error_threshhold)
				ternary_distances[i] = 0;
			else if(fabs(sensors_distances[i] - (min_dist + dist_to_back_panel)) < error_threshhold)
				ternary_distances[i] = 1;
			else if(sensors_distances[i] > min_dist + dist_to_back_panel)
				ternary_distances[i] = 2;
			else if((sensors_distances[i] < min_dist + dist_to_back_panel) && (sensors_distances[i] > min_dist))
			{
				ROS_INFO_STREAM("intermediate value detected; defaulting to detection of front panel");
				ternary_distances[i] = 0;
			}
			else
				ROS_INFO_STREAM("very confused");

		}

		bool cutout_found = false;
		for(int i = 0; i < NUM_SENSORS - 1; i++)
		{
			if((ternary_distances[i] == 1) && (ternary_distances[i+1] == 1))
			{
				if(cutout_found)
					ROS_ERROR_STREAM_THROTTLE(1, "two cutouts found????");
				else
					location = i;
				cutout_found = true;
			}
		}
		ROS_INFO_STREAM("0: " << ternary_distances[0] << " 1: " << ternary_distances[1] << " 2: " << ternary_distances[2] << " 3: " << ternary_distances[3]);
		ROS_INFO_STREAM("location = " << location);

		if(location == (NUM_SENSORS-1)/2)
		{
			ROS_INFO_STREAM("PERFECT");
			cmd_vel_msg.linear.x = 0;
		}
		else if(location < (NUM_SENSORS-1)/2)
		{
			ROS_INFO_STREAM("move left");
			cmd_vel_msg.linear.x = 0.2; // TODO : this might not be enough to get the robot to move?
		}
		else
		{
			ROS_INFO_STREAM("move right");
			cmd_vel_msg.linear.x = -0.2;
		}


		cmd_vel_pub.publish(cmd_vel_msg);

		ros::spinOnce();
		r.sleep();
	}

	return 0;
}
