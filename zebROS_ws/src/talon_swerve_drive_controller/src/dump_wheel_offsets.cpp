#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include "talon_state_controller/TalonState.h"
#include <fstream>
#include <sstream>

talon_state_controller::TalonState talon_state_msg;
std::vector<double> offsets;

bool get_offsets_srv(std_srvs::Trigger::Request& /*req*/, std_srvs::Trigger::Response& /*res*/)
{
	ROS_INFO_STREAM("running get_offsets_srv");
	std::ofstream offsets_file;
	std::stringstream offsets_file_name;
	offsets_file_name <<  "/home/ubuntu/2019RobotCode/zebROS_ws/src/ros_control_boilerplate/config/offsets_" << std::to_string(ros::Time::now().toSec()) << ".yaml";
	offsets_file.open(offsets_file_name.str());

	offsets_file << "swerve_drive_controller:\n";
	offsets_file << "\tsteering_joint_fl:\n";
	offsets_file << "\t" << "\t" << "offset: "<< talon_state_msg.position[0] << "\n";
	offsets_file << "\tsteering_joint_fr:"<< "\n";
	offsets_file << "\t" << "\t" << "offset: "<< talon_state_msg.position[2] << "\n";
	offsets_file << "\tsteering_joint_bl:"<< "\n";
	offsets_file << "\t" << "\t" << "offset: "<< talon_state_msg.position[4] << "\n";
	offsets_file << "\tsteering_joint_br:"<< "\n";
	offsets_file << "\t" << "\t" << "offset: "<< talon_state_msg.position[6] << "\n";

	return true;
}

void talon_states_cb(const talon_state_controller::TalonState& msg)
{
	talon_state_msg = msg;
}

int main(int argc, char ** argv)
{
	ros::init(argc, argv, "dump_wheel_offsets");
	ros::NodeHandle nh;

	ros::Subscriber talon_states_sub = nh.subscribe("/frcrobot_jetson/talon_states", 1, talon_states_cb);
	ros::ServiceServer offsets_srv = nh.advertiseService("dump_offsets", get_offsets_srv);

	ros::spin();

	return 0;
}
