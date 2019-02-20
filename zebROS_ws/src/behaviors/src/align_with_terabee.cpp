#include <ros/ros.h>
#include <vector>
#include "teraranger_array/RangeArray.h"
#include "geometry_msgs/Twist.h"
#include "std_srvs/SetBool.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float64.h"

#define NUM_SENSORS 4
#define MIN_DIST 4

std::vector<double> sensors_distances;
std::vector<int> ternary_distances;
bool publish = false;
bool publish_last = false;
double min_dist = 100000000000;

void multiflexCB(const teraranger_array::RangeArray& msg)
{
	min_dist = 1000000000;
	for(int i = 0; i < NUM_SENSORS; i++)
	{
		if(msg.ranges[i].range == msg.ranges[i].range)
		{
			sensors_distances[i] = msg.ranges[i].range;
			min_dist = std::min(min_dist, static_cast<double>(msg.ranges[i].range));
			//ROS_INFO_STREAM("i = " << i << " range = " << sensors_distances[i]);
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
	//ROS_INFO_STREAM("running/stopping align with terabee " << publish);
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
	double distance_target;

	if(!n_params.getParam("dist_to_back_panel", dist_to_back_panel))
		ROS_ERROR_STREAM("Could not read dist_to_back_panel in align_with_terabee");
	if(!n_params.getParam("cmd_vel_to_pub", cmd_vel_to_pub))
		ROS_ERROR_STREAM("Could not read cmd_vel_to_pub in align_with_terabee");
	if(!n_params.getParam("error_threshhold", error_threshhold))
		ROS_ERROR_STREAM("Could not read error_threshhold in align_with_terabee");
	if(!n_params.getParam("distance_target", distance_target))
		ROS_ERROR_STREAM("Could not read distance_target in align_with_terabee");


	sensors_distances.resize(NUM_SENSORS);
	ternary_distances.resize(NUM_SENSORS);

	ros::Publisher distance_setpoint_pub = n.advertise<std_msgs::Float64>("distance_pid/setpoint", 1);
	ros::Publisher distance_state_pub = n.advertise<std_msgs::Float64>("distance_pid/state", 1);
	ros::Publisher distance_enable_pub = n.advertise<std_msgs::Bool>("distance_pid/pid_enable", 1);
	ros::Publisher y_command_pub = n.advertise<std_msgs::Float64>("align_with_terabee/y_command", 1);
	ros::Publisher successful_y_align = n.advertise<std_msgs::Bool>("align_with_terabee/y_aligned", 1);

	ros::Subscriber terabee_sub = n.subscribe("/multiflex_1/ranges_raw", 1, &multiflexCB);
	ros::Subscriber start_stop_sub = n.subscribe("align_with_terabee/enable_y_pub", 1, &startStopCallback);

	ros::ServiceServer start_stop_service = n.advertiseService("align_with_terabee", startStopAlign);

	std_msgs::Float64 y_msg;
	y_msg.data = 0;

	std_msgs::Float64 distance_setpoint_msg;
	distance_setpoint_msg.data = distance_target;

	ros::Rate r(50);

	//make the robot not randomly drive forward as soon as it receives data
	std_msgs::Bool enable_false;
	enable_false.data = false;
	distance_enable_pub.publish(enable_false);
	ros::spinOnce();


	while(ros::ok())
	{
		bool aligned = false;
		if(sensors_distances[0] == 0.0 && sensors_distances[1] == 0.0 && sensors_distances[2] == 0.0 && sensors_distances[3] == 0.0)
		{
			ROS_INFO_STREAM_THROTTLE(2, "No data is being received from the Terabee sensors. Skipping this message");
			ros::spinOnce();
			r.sleep();
			continue;
		}

		//deal with distance PID first
		std_msgs::Float64 distance_state_msg;
		distance_state_msg.data = min_dist;
		distance_state_pub.publish(distance_state_msg);

		distance_setpoint_pub.publish(distance_setpoint_msg.data);

		//now the exciting y-alignment stuff
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
				//ROS_INFO_STREAM("intermediate value detected; defaulting to detection of front panel");
				ternary_distances[i] = 0;
			}
			else
			{
				//ROS_INFO_STREAM("very confused " << sensors_distances[i]);
			}

		}
		//ROS_INFO_STREAM("minimum_distance = " << min_dist);
		//ROS_INFO_STREAM("ternary_distances 0: " << ternary_distances[0] << " 1: " << ternary_distances[1] << " 2: " << ternary_distances[2] << " 3: " << ternary_distances[3]); 

		bool cutout_found = false;

		//the robot is aligned
		if(ternary_distances[0] == 0 && ternary_distances[1] == 1 && ternary_distances[2] == 1 && ternary_distances[3] == 0)
		{
			ROS_INFO_STREAM("ALIGNED");
			aligned = true;
			y_msg.data= 0.0;
			cutout_found = true;
		}

		//cutout seen in leftmost or rightmost two sensors, and panel seen in other two
		if(!cutout_found)
		{
			if(ternary_distances[0] == 0 && ternary_distances[1] == 0 && ternary_distances[2] == 1 && ternary_distances[3] == 1)
			{
				//ROS_INFO_STREAM("move right, either cargo or rocket");
				y_msg.data= -1*cmd_vel_to_pub;
				cutout_found = true;
			}
			else if(ternary_distances[0] == 1 && ternary_distances[1] == 1 && ternary_distances[2] == 0 && ternary_distances[3] == 0)
			{
				//ROS_INFO_STREAM("move left, either cargo or rocket");
				y_msg.data= cmd_vel_to_pub;
				cutout_found = true;
			}
		}

		//only one sensor senses the cutout, and then the other three see the panel
		if(!cutout_found)
		{
			if(ternary_distances[0] == 1 && (ternary_distances[1] == 0 && ternary_distances[2] == 0 && ternary_distances[3] == 0))
			{
				//ROS_INFO_STREAM("move left, probably cargo ship");
				y_msg.data= cmd_vel_to_pub;
				cutout_found = true;
			}
			else if (ternary_distances[0] == 0 && ternary_distances[1] == 0 && ternary_distances[2] == 0 && ternary_distances[3] == 1)
			{
				//ROS_INFO_STREAM("move right, probably cargo ship");
				y_msg.data= -1*cmd_vel_to_pub;
				cutout_found = true;
			}
		}

		//the robot senses a cutout with two sensors, and then the panel, and then really far
		if(!cutout_found)
		{
			if(ternary_distances[0] == 1 && (ternary_distances[1] == 1 && ternary_distances[2] == 0 && ternary_distances[3] == 2))
				{
				//ROS_INFO_STREAM("move left, probably rocket edge");
				y_msg.data= cmd_vel_to_pub;
				cutout_found = true;
			}
			else if (ternary_distances[0] == 2 && ternary_distances[1] == 0 && ternary_distances[2] == 1 && ternary_distances[3] == 1)
			{
				//ROS_INFO_STREAM("move right, probably rocket edge");
				y_msg.data= -1*cmd_vel_to_pub;
				cutout_found = true;
			}
		}

		//the robot senses a cutout with two sensors, and then the panel, and then what SEEMS like a cutout but is actually the other side of the rocket
		if(!cutout_found)
		{
			if(ternary_distances[0] == 1 && (ternary_distances[1] == 1 && ternary_distances[2] == 0 && ternary_distances[3] == 1))
			{
				//ROS_INFO_STREAM("move left, probably rocket center");
				y_msg.data= cmd_vel_to_pub;
				cutout_found = true;
			}
			else if (ternary_distances[0] == 1 && ternary_distances[1] == 0 && ternary_distances[2] == 1 && ternary_distances[3] == 1)
			{
				//ROS_INFO_STREAM("move right, probably rocket center");
				y_msg.data= -1*cmd_vel_to_pub;
				cutout_found = true;
			}
		}

		if(!cutout_found)
		{
			//ROS_INFO_STREAM("cutout not found; can't align");
			y_msg.data= 0;
		}

		if(publish)
		{
			y_command_pub.publish(y_msg);
		}
		else if(!publish && publish_last)
		{
			y_msg.data= 0;
			y_command_pub.publish(y_msg);
		}
		std_msgs::Bool aligned_msg;
		aligned_msg.data = aligned;
		successful_y_align.publish(aligned_msg);

		publish_last = publish;
		ros::spinOnce();
		r.sleep();
	}

	return 0;
}
