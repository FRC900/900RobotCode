#include <ros/ros.h>
#include <vector>
#include "teraranger_array/RangeArray.h"
#include "geometry_msgs/Twist.h"
#include "std_srvs/SetBool.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float64.h"

#define NUM_SENSORS 6
#define MIN_DIST 4

std::vector<double> sensors_distances;
int ternary_distances;
bool publish = false;
bool publish_last = false;
const double default_min_dist_ = 100;
double min_dist = default_min_dist_;

void multiflexCB(const teraranger_array::RangeArray& msg)
{
    min_dist = default_min_dist_;
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
			sensors_distances[i] = default_min_dist_;
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
	double distance_bound;
	double cmd_vel_to_pub;
	double distance_target;

	if(!n_params.getParam("cmd_vel_to_pub", cmd_vel_to_pub))
		ROS_ERROR_STREAM("Could not read cmd_vel_to_pub in align_with_terabee");
	if(!n_params.getParam("distance_bound", distance_bound))
		ROS_ERROR_STREAM("Could not read distance_bound in align_with_terabee");
	if(!n_params.getParam("distance_target", distance_target))
		ROS_ERROR_STREAM("Could not read distance_target in align_with_terabee");


	sensors_distances.resize(NUM_SENSORS);

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
	distance_setpoint_msg.data = 0;

	ros::Rate r(50);

	//make the robot not randomly drive forward as soon as it receives data
	std_msgs::Bool enable_false;
	enable_false.data = false;
	distance_enable_pub.publish(enable_false);
	ros::spinOnce();


	while(ros::ok())
	{
		bool aligned = false;
		if(sensors_distances[0] == 0.0 && sensors_distances[1] == 0.0 && sensors_distances[2] == 0.0 && sensors_distances[3] == 0.0 && sensors_distances[4] == 0.0)
		{
			ROS_INFO_STREAM("No data is being received from the Terabee sensors. Skipping this message");
			ros::spinOnce();
			r.sleep();
			continue;
		}

		//deal with distance PID first
        if(fabs(min_dist) < default_min_dist_) {
            std_msgs::Float64 distance_state_msg;
            distance_state_msg.data = min_dist - distance_target;
            distance_state_pub.publish(distance_state_msg);
            distance_setpoint_pub.publish(distance_setpoint_msg.data);
        }


		//now the exciting y-alignment stuff
		//1 is rocket panel
		//2 is empty space(can't differentiate on rocket between cutout and off the side)
		ternary_distances = 0;
		for(int i = 0; i < sensors_distances.size(); i++)
		{
			if(sensors_distances[i] != sensors_distances[i])
				ternary_distances += pow(10.0, NUM_SENSORS - 1 - i)*2;
			else if(fabs(sensors_distances[i] - min_dist) < distance_bound)
				ternary_distances += pow(10.0, NUM_SENSORS - 1 - i);  //TODO This is just i on compbot
			else if(fabs(sensors_distances[i] - min_dist) > distance_bound)
				ternary_distances += pow(10.0, NUM_SENSORS - 1 - i)*2;
			else
			{
				ROS_INFO_STREAM("very confused " << sensors_distances[i]);
			}

		}
		//ROS_INFO_STREAM("minimum_distance = " << min_dist);
		ROS_WARN_STREAM_THROTTLE(0.5, "ternary_distances: " << ternary_distances);

		bool cutout_found = false;


		switch(ternary_distances) {
			//the robot is aligned
			case(122212):
				ROS_INFO_STREAM_THROTTLE(.25, "The robot is aligned: case: " << ternary_distances);
				aligned = true;
				y_msg.data = 0;
				cutout_found = true;
				break;
			//Off to the right a small amount
			case(112212):
			case(212212):
			case(212211):
			case(211211):
			case(211221):
			case(221221):
			case(221121):
			case(222121):
				ROS_INFO_STREAM_THROTTLE(.25, "Off to the right a small amount: case: " << ternary_distances);
				cutout_found = true;
				y_msg.data = 1*cmd_vel_to_pub;
				break;
			//Off to the right a large amount
			case(221122):
			case(222122):
			case(222112):
			case(222212):
			case(222211):
			case(222221):
				ROS_INFO_STREAM_THROTTLE(.25, "Off to the right a large amount: case: " << ternary_distances);
				cutout_found = true;
				y_msg.data = 2*cmd_vel_to_pub;
				break;
			//Off to the left a small amount
			case(122112):
			case(122122):
			case(121122):
			//case(222122): Shouldn't happen
			//case(221122): Shouldn't happen now moved middle sensor to the right
			case(221222):
				ROS_INFO_STREAM_THROTTLE(.25, "Off to the left a small amount: case: " << ternary_distances);
				cutout_found = true;
				y_msg.data = -1*cmd_vel_to_pub;
				break;
			//Off to the left a large amount
			case(211122):
			case(211222):
			case(212222):
			case(112222):
			case(122222):
				ROS_INFO_STREAM_THROTTLE(.25, "Off to the left a large amount: case: " << ternary_distances);
				cutout_found = true;
				y_msg.data = -2*cmd_vel_to_pub;
				break;
		}

		if(!cutout_found)
		{
			ROS_INFO_STREAM_THROTTLE(.25, "cutout not found; can't align");
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
