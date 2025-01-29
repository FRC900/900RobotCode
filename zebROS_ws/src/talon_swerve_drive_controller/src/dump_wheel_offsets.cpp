#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include "talon_state_msgs/TalonFXProState.h"
#include <fstream>
#include <angles/angles.h>

/*
Procedure
- Bevel facing right
- Zero all encoder offsets
- Run this service
- Move resulting offsets_*.yaml file to correct robot name
*/

// TODO : make these config items, maybe?
const std::set<std::string> offset_joint_names{
	"bl_angle", 
	"br_angle", 
	"fl_angle", 
	"fr_angle", 
};
std::map<const std::string, double> joint_position_map;

bool get_offsets_srv(std_srvs::Trigger::Request& /*req*/, std_srvs::Trigger::Response& res)
{
	ROS_INFO_STREAM("running get_offsets_srv");
	std::stringstream offsets_file_name;
	offsets_file_name <<  "/home/ubuntu/900RobotCode/zebROS_ws/src/ros_control_boilerplate/config/offsets_" << std::to_string(ros::Time::now().toSec()) << ".yaml";
	std::ofstream offsets_file(offsets_file_name.str());

	for (const auto& joint_name: offset_joint_names)
	{
		const auto o = joint_position_map.find(joint_name); // lookup each joint's position by name
		if (o != joint_position_map.cend())
		{
			const auto offset = angles::normalize_angle(-o->second);

			ROS_INFO_STREAM("JOINT: " << joint_name << " OFFSET: " << offset);
			offsets_file << joint_name[0] << joint_name[1] << "_cancoder_controller:" << std::endl;
			offsets_file << "    type:       cancoder_controller/CANCoderController" << std::endl;
			offsets_file << "    joint_name: bl_cancoder" << std::endl; // WRONG SHOULD NOT BE BL_CANCODER 
			offsets_file << "    magnet_offset: " << offset << std::endl;
			offsets_file << "    dynamic_reconfigure: False" << std::endl;
			offsets_file << "    enable_read_thread: False" << std::endl;
			offsets_file << std::endl;
		}
		else
		{
			ROS_WARN_STREAM("Could not find joint name " << joint_name << " in talonfxpro_states");
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
	ros::ServiceServer offsets_srv = nh.advertiseService("dump_offsets", get_offsets_srv);

	ros::spin();

	return 0;
}
