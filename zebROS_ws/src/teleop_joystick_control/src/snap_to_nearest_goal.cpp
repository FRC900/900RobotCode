#include <atomic>
#include "std_srvs/Empty.h"
#include "sensor_msgs/Imu.h"
#include "ros/ros.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include "angles/angles.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"


#include <vector>

ros::Publisher snapAnglePub;
std::atomic<double> navX_angle;

std::vector<double> hatch_panel_angles;
std::vector<double> cargo_angles;
std::vector<double> nothing_angles;

double nearest_angle(std::vector<double> angles)
{
	double snap_angle;
	double smallest_distance = std::numeric_limits<double>::max();
	double cur_angle = angles::normalize_angle_positive(navX_angle.load(std::memory_order_relaxed));
	for(int i = 0; i < angles.size(); i++){
		double distance = fabs(angles::shortest_angular_distance(cur_angle, angles[i]));
		if(distance < smallest_distance) {
			smallest_distance = distance;
			snap_angle = angles[i];
		}
	}
	return snap_angle;
}

void navXCallback(const sensor_msgs::Imu &navXState)
{
    const tf2::Quaternion navQuat(navXState.orientation.x, navXState.orientation.y, navXState.orientation.z, navXState.orientation.w);
    double roll;
    double pitch;
    double yaw;
    tf2::Matrix3x3(navQuat).getRPY(roll, pitch, yaw);

    if (yaw == yaw) // ignore NaN results
        navX_angle.store(yaw, std::memory_order_relaxed);
	
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "Joystick_controller");
	ros::NodeHandle n;
	ros::NodeHandle n_params(n, "goal_angles");

	if(!n_params.getParam("hatch_panel_angles", hatch_panel_angles))
	{
		ROS_ERROR("Could not read hatch_panel_angles in teleop_joystick_comp");
	}
	if(!n_params.getParam("cargo_angles", cargo_angles))
	{
		ROS_ERROR("Could not read cargo_angles in teleop_joystick_comp");
	}
	if(!n_params.getParam("nothing_angles", nothing_angles))
	{
		ROS_ERROR("Could not read nothing_angles in teleop_joystick_comp");
	}


	navX_angle = M_PI / 2;

	ros::Subscriber navX_heading  = n.subscribe("/frcrobot_rio/navx_mxp", 1, &navXCallback);
	ros::Publisher snapAnglePub = n.advertise<std_msgs::Float64MultiArray>("snap_angle_pub", 10);
	ROS_INFO("snap_to_angle_init");
	
	ros::Rate r(10);
	bool has_hatch_panel = false;
	bool has_cargo = true;
	double snap_angle;
	while(ros::ok()) {
		std_msgs::Float64MultiArray angle_snap;
		if(has_hatch_panel) {
			snap_angle = nearest_angle(hatch_panel_angles);
		}
		else if(has_cargo) {
			snap_angle = nearest_angle(cargo_angles);
		}
		else {
			snap_angle = nearest_angle(nothing_angles);
		}
		angle_snap.data.push_back(snap_angle);
		angle_snap.data.push_back(angles::normalize_angle_positive(navX_angle.load(std::memory_order_relaxed)));
		snapAnglePub.publish(angle_snap);
		
		r.sleep();
		ros::spinOnce();
	}
	return 0;
}
