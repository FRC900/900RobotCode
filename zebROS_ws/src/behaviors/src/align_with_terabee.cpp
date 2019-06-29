#include <ros/ros.h>
#include <vector>
#include "teraranger_array/RangeArray.h"
#include "geometry_msgs/Twist.h"
#include "std_srvs/SetBool.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include <atomic>

#define NUM_SENSORS 8

// TODO - if we redid the interface to this to match the interface for a PID
// controller it would be a lot easier to make the align server code common
// between various modes. That is, make a debug, enable, etc message and populate
// them as-if this were the standard ROS PID node.  This would remove the need
// for special cases in the align server itself
//   Note - now done for the result - turned it into a float64 array like
//          the PID nodes so the align server can use common code to decode it
// TODO - also split up into two separate align with terabee nodes - one for
// cargo, one for hatch?  If not, have two interfaces matching PID controllers,
// one for cargo, one for hatch

std::vector<double> sensors_distances;
std::vector<double> adjust_terabee_dists;
bool publish = false;
bool publish_last = false;

const double default_min_dist_ = 1.5;
std::atomic<double> min_dist;
std::atomic<double> min_dist_cargo;
double min_dist_local;
double min_dist_cargo_local;

void multiflexCB(const teraranger_array::RangeArray& msg)
{
    min_dist_local = default_min_dist_;
	min_dist_cargo_local = default_min_dist_;
	for(int i = 0; i < NUM_SENSORS; i++)
	{
		if(msg.ranges[i].range == msg.ranges[i].range)
		{
			sensors_distances[i] = msg.ranges[i].range + adjust_terabee_dists[i];
			if(msg.ranges[i].range > default_min_dist_) {
				sensors_distances[i] = default_min_dist_ - .1;
			}
			if(i <= 1) {
				min_dist_cargo_local = std::min(min_dist_cargo_local, static_cast<double>(sensors_distances[i]));
			}
			if(i == 2 || i == 3) {
				min_dist_local = std::min(min_dist_local, static_cast<double>(sensors_distances[i]));
			}
			//ROS_INFO_STREAM("i = " << i << " range = " << sensors_distances[i]);
		}
		else
		{
			sensors_distances[i] = default_min_dist_;
		}
	}
	if(min_dist_cargo_local != default_min_dist_) {
		min_dist_cargo = min_dist_cargo_local;
	}
	if(min_dist_local != default_min_dist_) {
		min_dist = min_dist_local;
	}
}

bool startStopAlign(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
	publish = req.data;
	res.success = true;
	//ROS_INFO_STREAM("running/stopping align with terabee " << publish);
	// TODO : should probably return true here if there's no error
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

	double distance_bound;
	double cmd_vel_to_pub;
	double distance_target;
	double cargo_pid_max_distance;

	double last_command_published = 0.0;

	if(!n_params.getParam("adjust_terabee_dists", adjust_terabee_dists))
		ROS_ERROR_STREAM("Could not read adjust_terabee_dists in align_with_terabee");
	if(!n_params.getParam("cmd_vel_to_pub", cmd_vel_to_pub))
		ROS_ERROR_STREAM("Could not read cmd_vel_to_pub in align_with_terabee");
	if(!n_params.getParam("distance_bound", distance_bound))
		ROS_ERROR_STREAM("Could not read distance_bound in align_with_terabee");
	if(!n_params.getParam("distance_target", distance_target))
		ROS_ERROR_STREAM("Could not read distance_target in align_with_terabee");
	if(!n_params.getParam("cargo_pid_max_distance", cargo_pid_max_distance))
		ROS_ERROR_STREAM("Could not read cargo_pid_max_distance in align_with_terabee");

	sensors_distances.resize(NUM_SENSORS);

	ros::Publisher hatch_panel_distance_setpoint_pub = n.advertise<std_msgs::Float64>("hatch_panel_distance_pid/setpoint", 1);
	ros::Publisher hatch_panel_distance_state_pub = n.advertise<std_msgs::Float64>("hatch_panel_distance_pid/state", 1);
	ros::Publisher cargo_distance_state_pub = n.advertise<std_msgs::Float64>("cargo_distance_pid/state", 1);
	ros::Publisher cargo_distance_setpoint_pub = n.advertise<std_msgs::Float64>("cargo_distance_pid/setpoint", 1);
	ros::Publisher cargo_setpoint_pub = n.advertise<std_msgs::Float64>("cargo_pid/setpoint", 1);
	ros::Publisher cargo_state_pub = n.advertise<std_msgs::Float64>("cargo_pid/state", 1);
	ros::Publisher y_command_pub = n.advertise<std_msgs::Float64>("align_with_terabee/y_command", 1);
	ros::Publisher successful_y_align = n.advertise<std_msgs::Float64MultiArray>("align_with_terabee/y_aligned", 1);

	ros::Subscriber terabee_sub = n.subscribe("/multiflex_1/ranges_raw", 1, &multiflexCB);
	ros::Subscriber start_stop_sub = n.subscribe("align_with_terabee/enable_y_pub", 1, &startStopCallback);

	ros::ServiceServer start_stop_service = n.advertiseService("align_with_terabee", startStopAlign);

	std_msgs::Float64 y_msg;
	y_msg.data = 0;

	std_msgs::Float64 distance_setpoint_msg;
	distance_setpoint_msg.data = 0;

	std_msgs::Float64 cargo_setpoint_msg;
	cargo_setpoint_msg.data = 0;

	ros::Rate r(100);

	while(ros::ok())
	{
		bool aligned = false;
		ros::spinOnce();
		if(sensors_distances[0] == 0.0 && sensors_distances[1] == 0.0 &&sensors_distances[2] == 0.0)
		{
			ROS_INFO_STREAM_THROTTLE(2, "No data is being received from the Terabee sensors. Skipping this message");
			ros::spinOnce();
			r.sleep();
			continue;
		}

		//ROS_ERROR_STREAM_THROTTLE(0.25, "min_dist: " << min_dist);

		//deal with distance PID first
		ros::spinOnce();
        if(fabs(min_dist) < default_min_dist_) {
            std_msgs::Float64 distance_state_msg;
            distance_state_msg.data = min_dist - distance_target;
            hatch_panel_distance_state_pub.publish(distance_state_msg);
            hatch_panel_distance_setpoint_pub.publish(distance_setpoint_msg);
        }
		ros::spinOnce();
		if(fabs(min_dist_cargo) < default_min_dist_) {
            std_msgs::Float64 distance_state_msg;
            distance_state_msg.data = distance_target - min_dist_cargo;
            cargo_distance_state_pub.publish(distance_state_msg);
            cargo_distance_setpoint_pub.publish(distance_setpoint_msg);
		}

		//deal with cargo PID next
		std_msgs::Float64 cargo_state_msg;
		double dist_left = sensors_distances[0];
		double dist_right = sensors_distances[1];
		if(dist_left != dist_left)
			dist_left = 1.0;
		if(dist_right != dist_right)
			dist_right = 1.0;
		dist_left = std::max(dist_left, 0.0);
		dist_right = std::max(dist_right, 0.0);
		dist_left = std::min(dist_left, cargo_pid_max_distance);
		dist_right = std::min(dist_right, cargo_pid_max_distance);

		cargo_state_msg.data = dist_left - dist_right;
		cargo_state_pub.publish(cargo_state_msg);
		cargo_setpoint_pub.publish(cargo_setpoint_msg);
		ros::spinOnce();

		//now the exciting y-alignment stuff
		//1 is wall
		//2 is empty space(can't differentiate on rocket between cutout and off the side)
		int ternary_distances = 0;
		for(size_t i = 2; i < sensors_distances.size(); i++)
		{
			if(sensors_distances[i] != sensors_distances[i])
				ternary_distances += pow(10.0, i - 2)*2;
			else if(fabs(sensors_distances[i] - min_dist) < distance_bound)
				ternary_distances += pow(10.0, i - 2);
			else if(fabs(sensors_distances[i] - min_dist) > distance_bound)
				ternary_distances += pow(10.0, i - 2)*2;
			else
			{
				//ROS_INFO_STREAM_THROTTLE(1,"index " << i << " is very confused " << sensors_distances[i]);
			}
		}
		//ROS_INFO_STREAM("minimum_distance = " << min_dist);
		//ROS_WARN_STREAM_THROTTLE(0.5, "ternary_distances: " << ternary_distances);

		bool cutout_found = false;
		switch(ternary_distances) {
			//the robot is aligned or close enough
			case(122212):
				ROS_INFO_STREAM_THROTTLE(.25, "The robot is aligned: case: " << ternary_distances);
				aligned = true;
				y_msg.data = 0;
				cutout_found = true;
				break;
			//Off to the right a small amount
			//case(212212):
			case(112212): //off to the right a small amount, but really close
				if(last_command_published > 0.0) { //if last command was positive to move closer to center set aligned to not overshoot
					cutout_found = true;
					y_msg.data = 0;
					aligned = true;
					break;
				}
			case(112211):
			case(111211):
			case(211221):
			case(212211): //Just added technically shouldn't happen
			case(221221):
			case(221121):
			case(222121):
			case(222112):
			case(222122): //DUPLICATE moving sensor 4 one to the right makes this only case
				ROS_INFO_STREAM_THROTTLE(.25, "Off to the right a small amount: case: " << ternary_distances);
				cutout_found = true;
				y_msg.data = 1*cmd_vel_to_pub;
				break;
			//Off to the right a large amount
			case(222212):
			case(222211):
			case(222221):
				ROS_INFO_STREAM_THROTTLE(.25, "Off to the right a large amount: case: " << ternary_distances);
				cutout_found = true;
				y_msg.data = 2*cmd_vel_to_pub;
				break;
			//Off to the left a small amount
			case(122112): //off to the left a small amount, but really close
				/*  //Commented out because same as off to the left more than a few inches on cargo ship, so can't set aligned without knowing cargo or rocket
				if(last_command_published < 0.0) { //if last command was positive to move closer to center set aligned to not overshoot
					cutout_found = true;
					y_msg.data = 0;
					aligned = true;
					break;
				}
				*/
			case(122122):
			//case(222122): //Shouldn't happen
			case(211122):
			case(221222):
				ROS_INFO_STREAM_THROTTLE(.25, "Off to the left a small amount: case: " << ternary_distances);
				cutout_found = true;
				y_msg.data = -1*cmd_vel_to_pub;
				break;
			//Off to the left a large amount
			case(211222):
			case(111222):
			case(112222):
			case(122222):
				ROS_INFO_STREAM_THROTTLE(.25, "Off to the left a large amount: case: " << ternary_distances);
				cutout_found = true;
				y_msg.data = -2*cmd_vel_to_pub;
				break;


			//Cargo ship cases
			//Aligned
			case(122211):
				ROS_INFO_STREAM_THROTTLE(.25, "The robot is aligned with cargo ship: case: " << ternary_distances);
				aligned = true;
				y_msg.data = 0;
				cutout_found = true;
				break;
			//Off to the right
				/*    //Comented out because duplicate of of the right more than a few inches on rocket so can't set aligned for this case without knowing rocket or caro
			case(112211): //off a bit but really close
				if(last_command_published > 0.0) { //if last command was positive to move closer to center set aligned to not overshoot
					cutout_found = true;
					y_msg.data = 0;
					aligned = true;
					break;
				}
				*/
			case(112221):
			case(111112): //off to the right at loading station
				ROS_INFO_STREAM_THROTTLE(.25, "Off to the right of cargo ship a small amount: case: " << ternary_distances);
				cutout_found = true;
				y_msg.data = 1*cmd_vel_to_pub;
				break;
			//Off to the right a lot need to check if this is important for cargo ship placing as this happens on the rocket
			//case(111221):
			case(111121):
			case(211121):
				ROS_INFO_STREAM_THROTTLE(.25, "Off to the right of cargo ship a large amount: case: " << ternary_distances);
				cutout_found = true;
				y_msg.data = 2*cmd_vel_to_pub;
				break;
			//Off to the left
			case(122111): //off a bit but really close
				/*TODO commented out because can go many inches off to the left in this case probably will cause severe undershooting.
				if(last_command_published < 0.0) { //if last command was positive to move closer to center set aligned to not overshoot
					cutout_found = true;
					y_msg.data = 0;
					aligned = true;
					break;
				}*/
			//case(122112): //Duplicate of off to the left of rocket a small amount
			case(222111):
				ROS_INFO_STREAM_THROTTLE(.25, "Off to the left of cargo ship a small amount: case: " << ternary_distances);
				cutout_found = true;
				y_msg.data = -1*cmd_vel_to_pub;
				break;
			case(211112):
				ROS_INFO_STREAM_THROTTLE(.25, "Off to the left of cargo ship a large amount: case: " << ternary_distances);
				cutout_found = true;
				y_msg.data = -2*cmd_vel_to_pub;
				break;
			//Indeterminate cases
			case(221122):
			case(221112):
			case(221111): //Off to the left a bit at loading station but shows up to the right on rocket
				ROS_INFO_STREAM_THROTTLE(.25, "Indeterminate case found: case: " << ternary_distances);
				cutout_found = true;
				y_msg.data = last_command_published;
				break;
			default:
				break;
		}

		if(!cutout_found)
		{
			//ROS_INFO_STREAM_THROTTLE(.25, "cutout not found; can't align");
			//Don't publish anything when not found to let previous commands drift the bot a bit
			//y_msg.data= 0;
		}
		if(publish)
		{
			y_command_pub.publish(y_msg);
			last_command_published = y_msg.data;
		}
		else if(!publish && publish_last)
		{
			y_msg.data= 0;
			y_command_pub.publish(y_msg);
		}
		// Match the PID debug output format
		// The 0th entry in the array is the error. Since we only
		// have binary thresholding here, set that "error" to either 0
		// or some huge number depending on whether or not we are or
		// are not aligned
		std_msgs::Float64MultiArray aligned_msg;
		aligned_msg.data.resize(5);
		aligned_msg.data[0] = aligned ? 0.0 : std::numeric_limits<double>::max();
		successful_y_align.publish(aligned_msg);

		publish_last = publish;
		ros::spinOnce();
		r.sleep();
	}

	return 0;
}
