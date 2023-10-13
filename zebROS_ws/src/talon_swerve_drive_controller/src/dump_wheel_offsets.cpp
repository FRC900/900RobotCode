#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include "talon_state_msgs/TalonFXProState.h"
#include <fstream>
#include <angles/angles.h>

/*
Procedure
- Bevel facing right
- Run old dump offsets  
- 2pi - each offset
- that + pi
*/


// TODO : make these config items, maybe?
const std::map<const std::string, const std::string> offset_joint_names{
	{"fl_angle", "steering_joint_fl"},
	{"fr_angle", "steering_joint_fr"},
	{"bl_angle", "steering_joint_bl"},
	{"br_angle", "steering_joint_br"}
};
std::map<const std::string, double> joint_position_map;

#if 0
talon_state_msgs::TalonConfig talon_config_msg;
void talon_config_cb(const talon_state_msgs::TalonConfig& msg)
{
	talon_config_msg = msg;
}
#endif


bool get_offsets_srv(std_srvs::Trigger::Request& /*req*/, std_srvs::Trigger::Response& res)
{
	ROS_INFO_STREAM("running get_offsets_srv");
	std::stringstream offsets_file_name;
	offsets_file_name <<  "/home/ubuntu/2023RobotCode/zebROS_ws/src/ros_control_boilerplate/config/offsets_" << std::to_string(ros::Time::now().toSec()) << ".yaml";
	std::ofstream offsets_file(offsets_file_name.str());

	offsets_file << "swerve_drive_controller:" << std::endl;
	for (const auto& joint : offset_joint_names)
	{
		const auto o = joint_position_map.find(joint.first); // lookup by name
		if (o != joint_position_map.cend())
		{
			double offset;
			offset = angles::normalize_angle(o->second - M_PI / 2.);
#if 0
			if (talon_config_msg.invert[i]) {
				offset = angles::normalize_angle(M_PI - (o->second - M_PI / 2.0));
			}
#endif

			// Subtracing from pi because that is what worked before
			ROS_INFO_STREAM("JOINT: " << joint.first << " OFFSET: " << offset);
			offsets_file << "    " << joint.second << ":" << std::endl;
			offsets_file << "        offset: " << offset << std::endl;
		}
		else{
			ROS_WARN_STREAM("Could not find joint name " << joint.first << " in talonfxpro_states");
		}
	}
	res.success = true;

	return true;
}

void talon_states_cb(const talon_state_msgs::TalonFXProState& msg)
{
	for (size_t i = 0; i < msg.name.size(); i++)
	{
		if (offset_joint_names.count(msg.name[i]) > 0)
		{
			joint_position_map[msg.name[i]] = msg.position[i];
		}
	}
}

int main(int argc, char ** argv)
{
	ros::init(argc, argv, "dump_wheel_offsets");
	ros::NodeHandle nh;

	ros::Subscriber talon_states_sub = nh.subscribe("/frcrobot_jetson/talonfxpro_states", 1, talon_states_cb);
	//ros::Subscriber talon_configs_sub = nh.subscribe("/frcrobot_jetson/talon_configs", 1, talon_config_cb);
	ros::ServiceServer offsets_srv = nh.advertiseService("dump_offsets", get_offsets_srv);

	ros::spin();

	return 0;
}
