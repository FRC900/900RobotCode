#include <ros/ros.h>

/*
 * Publishes orient state, orient setpoint, y state, y setpoint, and pid_enable
 */

/* The logic for stopping and starting PID should probably be put in actionlib, not here */

#include "std_msgs/Float64.h"
#include "teraranger_array/RangeArray.h"
#include "std_msgs/Bool.h"
#include "std_srvs/SetBool.h"
#include "sensor_msgs/Imu.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

// TODO : nit, but explicitily initializing to false
// will make it obvious what the intended use of this is
bool publish;

double navx_angle_state;
double distance_state;

 // TODO : all of these can be locals - in general
 // put declarations as close to their use as possible
 // Makes it easer to keep track of what's used for what
std_msgs::Float64 orient_state_msg;
std_msgs::Float64 y_state_msg;
std_msgs::Float64 orient_setpoint_msg;
std_msgs::Float64 y_target_msg;
std_msgs::Bool pid_enable_msg;

// TODO : same with these
ros::Subscriber world_vector_sub;
ros::Publisher orient_setpoint;
ros::Publisher orient_state;
ros::Publisher y_setpoint;
ros::Publisher y_state;
ros::Publisher pid_enable;
ros::ServiceServer run_align;

double distance_between_sensors = 0.3937;

void distanceCB(const teraranger_array::RangeArray& msg)
{
	if(msg.ranges[1].range == msg.ranges[1].range && msg.ranges[5].range == msg.ranges[5].range)
		// TODO: Consider using atan2?
		distance_state = atan((msg.ranges[1].range - msg.ranges[5].range) / distance_between_sensors);
	else
		ROS_WARN_STREAM("at least one of the distance centers is NAN");
}

// TODO : not sure which joint this is, but I don't know we directly
// publish navX angle to joint states. There's an offset, which is the
// value used to zero navX readings, but I don't think that is what we
// want here. Check out the navX callback in the swerve drive controller
// to see how to read navX heading
void jointStatesCB(const sensor_msgs::Imu& msg)
{
    const tf2::Quaternion navQuat(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w);
    double roll;
    double pitch;
    double yaw;
    tf2::Matrix3x3(navQuat).getRPY(roll, pitch, yaw);

    if (yaw == yaw) // ignore NaN results
		navx_angle_state = yaw;
}

bool align_service(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
	publish = req.data;
	res.success = true;
	return true;
}

int main(int argc, char ** argv)
{
	ros::init(argc, argv, "distance_setpoint_pub");
	ros::NodeHandle nh;
	int sensor_used = 1; //0 for navx, 1 for distance sensors, sorry!!

	//the orientation we want is 0
	orient_setpoint_msg.data = 0;

	//state subscribers
	ros::Subscriber distance_sub = nh.subscribe("/multiflex_1/ranges_raw", 1, distanceCB);
	ros::Subscriber angle_sub = nh.subscribe("/imu/zeroed_imu", 1, jointStatesCB);

	//required pid_enable topic for the pid node
	// TODO : consider how namespaces are doing to work, since I assume
	// we'll eventually have multiple PID controllers running.
	// It might be easy enough to set up using namespaces, but
	// prepare for it now?
	ros::Publisher pid_enable = nh.advertise<std_msgs::Bool>("pid_enable", 1);

	//sets true/false to PID node
	ros::ServiceServer run_align = nh.advertiseService("run_align", align_service);

	//setpoint and state publishers
	// TODO : all of these should be locals
	ros::Publisher orient_setpoint = nh.advertise<std_msgs::Float64>("orient_setpoint", 1);
	ros::Publisher orient_state = nh.advertise<std_msgs::Float64>("orient_state", 1);
	ros::Publisher y_setpoint = nh.advertise<std_msgs::Float64>("y_setpoint", 1);
	ros::Publisher y_state = nh.advertise<std_msgs::Float64>("y_state", 1);
	ros::Publisher navx_setpoint = nh.advertise<std_msgs::Float64>("navx_setpoint", 1);
	ros::Publisher navx_state = nh.advertise<std_msgs::Float64>("navx_state", 1);

	ros::Rate r(100);

	while(ros::ok())
	{
		orient_setpoint.publish(orient_setpoint_msg);

		switch (sensor_used)
		{
			case 0:
				orient_state_msg.data = navx_angle_state;
				break;
			case 1:
				orient_state_msg.data = distance_state;
				break;
			default:
				orient_state_msg.data = 0;
				break;
		}

		orient_state.publish(orient_state_msg);

		pid_enable_msg.data = publish;
		pid_enable.publish(pid_enable_msg);

		ros::spinOnce();
		r.sleep();
	}

	return 0;
}
