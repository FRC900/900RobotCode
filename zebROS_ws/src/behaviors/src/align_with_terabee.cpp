#include <ros/ros.h>
#include <vector>
#include "teraranger_array/RangeArray.h"
#include "geometry_msgs/Twist.h"
#include "std_srvs/SetBool.h"
#include "std_msgs/Bool.h"

#define NUM_SENSORS 4
#define MIN_DIST 4

std::vector<double> sensors_distances;
std::vector<int> ternary_distances;
bool publish = false;
bool publish_last = false;
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
	min_dist = 1000000000;
	for(int i = 0; i < NUM_SENSORS; i++)
	{
		if(msg.ranges[i].range == msg.ranges[i].range)
		{
			sensors_distances[i] = msg.ranges[i].range;
			min_dist = std::min(min_dist, static_cast<double>(msg.ranges[i].range));
			ROS_INFO_STREAM("i = " << i << " range = " << sensors_distances[i]);
		}
		else
		{
			sensors_distances[i] = 1000000000;
		}
	}
}

bool startStopAlign(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
	publish = req.data;
	res.success = true;
	ROS_INFO_STREAM("running/stopping align with terabee " << publish);
	return 0;
}

void startStopCallback(std_msgs::Bool msg)
{
	publish = msg.data;
}

int main(int argc, char ** argv)
{
	ros::init(argc, argv, "align_with_terabee");
	ros::NodeHandle n;
	ros::NodeHandle n_params(n, "align_with_terabee_params");

	double dist_to_back_panel;
	double error_threshhold;
	double cmd_vel_to_pub;

	if(!n_params.getParam("dist_to_back_panel", dist_to_back_panel))
		ROS_ERROR_STREAM("Could not read dist_to_back_panel in align_with_terabee");
	if(!n_params.getParam("cmd_vel_to_pub", cmd_vel_to_pub))
		ROS_ERROR_STREAM("Could not read cmd_vel_to_pub in align_with_terabee");
	if(!n_params.getParam("error_threshhold", error_threshhold))
		ROS_ERROR_STREAM("Could not read error_threshhold in align_with_terabee");


	sensors_distances.resize(NUM_SENSORS);
	ternary_distances.resize(NUM_SENSORS);

	ros::Subscriber terabee_sub = n.subscribe("/multiflex_1/ranges_raw", 1, &multiflexCB);
	ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("swerve_drive_controller/cmd_vel", 1);
	ros::ServiceServer start_stop_service = n.advertiseService("align_with_terabee", startStopAlign);
	ros::Subscriber start_stop_sub = n.subscribe("align_with_terabee_pub", 1, &startStopCallback);

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
		if(sensors_distances[0] == 0.0 && sensors_distances[1] == 0.0 && sensors_distances[2] == 0.0 && sensors_distances[3] == 0.0)
		{
			ROS_INFO_STREAM_THROTTLE(1, "No data is being received from the Terabee sensors. Skipping this message");
			ros::spinOnce();
			r.sleep();
			continue;
		}

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
			{
				ROS_INFO_STREAM("very confused " << sensors_distances[i]);
			}

		}
		ROS_INFO_STREAM("minimum_distance = " << min_dist);
		ROS_INFO_STREAM("ternary_distances 0: " << ternary_distances[0] << " 1: " << ternary_distances[1] << " 2: " << ternary_distances[2] << " 3: " << ternary_distances[3]); 

		bool cutout_found = false;

		//the robot is aligned
		if(ternary_distances[0] == 0 && ternary_distances[1] == 1 && ternary_distances[2] == 1 && ternary_distances[3] == 0)
		{
			ROS_INFO_STREAM("ALIGNED");
			cmd_vel_msg.linear.x = 0.0;
			cutout_found = true;
		}

		//cutout seen in leftmost or rightmost two sensors, and panel seen in other two
		if(!cutout_found)
		{
			if(ternary_distances[0] == 0 && ternary_distances[1] == 0 && ternary_distances[2] == 1 && ternary_distances[3] == 1)
			{
				ROS_INFO_STREAM("move right, either cargo or rocket");
				cmd_vel_msg.linear.x = -1*cmd_vel_to_pub;
				cutout_found = true;
			}
			else if(ternary_distances[0] == 1 && ternary_distances[1] == 1 && ternary_distances[2] == 0 && ternary_distances[3] == 0)
			{
				ROS_INFO_STREAM("move left, either cargo or rocket");
				cmd_vel_msg.linear.x = cmd_vel_to_pub;
				cutout_found = true;
			}
		}

		//only one sensor senses the cutout, and then the other three see the panel
		if(!cutout_found)
		{
			if(ternary_distances[0] == 1 && (ternary_distances[1] == 0 && ternary_distances[2] == 0 && ternary_distances[3] == 0))
			{
				ROS_INFO_STREAM("move left, probably cargo ship");
				cmd_vel_msg.linear.x = cmd_vel_to_pub;
				cutout_found = true;
			}
			else if (ternary_distances[0] == 0 && ternary_distances[1] == 0 && ternary_distances[2] == 0 && ternary_distances[3] == 1)
			{
				ROS_INFO_STREAM("move right, probably cargo ship");
				cmd_vel_msg.linear.x = -1*cmd_vel_to_pub;
				cutout_found = true;
			}
		}

		//the robot senses a cutout with two sensors, and then the panel, and then really far
		if(!cutout_found)
		{
			if(ternary_distances[0] == 1 && (ternary_distances[1] == 1 && ternary_distances[2] == 0 && ternary_distances[3] == 2))
				{
				ROS_INFO_STREAM("move left, probably rocket edge");
				cmd_vel_msg.linear.x = cmd_vel_to_pub;
				cutout_found = true;
			}
			else if (ternary_distances[0] == 2 && ternary_distances[1] == 0 && ternary_distances[2] == 1 && ternary_distances[3] == 1)
			{
				ROS_INFO_STREAM("move right, probably rocket edge");
				cmd_vel_msg.linear.x = -1*cmd_vel_to_pub;
				cutout_found = true;
			}
		}

		//the robot senses a cutout with two sensors, and then the panel, and then what SEEMS like a cutout but is actually the other side of the rocket
		if(!cutout_found)
		{
			if(ternary_distances[0] == 1 && (ternary_distances[1] == 1 && ternary_distances[2] == 0 && ternary_distances[3] == 1))
			{
				ROS_INFO_STREAM("move left, probably rocket center");
				cmd_vel_msg.linear.x = cmd_vel_to_pub;
				cutout_found = true;
			}
			else if (ternary_distances[0] == 1 && ternary_distances[1] == 0 && ternary_distances[2] == 1 && ternary_distances[3] == 1)
			{
				ROS_INFO_STREAM("move right, probably rocket center");
				cmd_vel_msg.linear.x = -1*cmd_vel_to_pub;
				cutout_found = true;
			}
		}

		if(!cutout_found)
		{
			ROS_INFO_STREAM("cutout not found; can't align");
			cmd_vel_msg.linear.x = 0;
		}

		if(publish)
		{
			cmd_vel_pub.publish(cmd_vel_msg);
		}
		else if(!publish && publish_last)
		{
			cmd_vel_msg.linear.x = 0;
			cmd_vel_pub.publish(cmd_vel_msg);
		}
		ROS_INFO_STREAM("publish = " << publish);

		publish_last = publish;
		ros::spinOnce();
		r.sleep();
	}

	return 0;
}
