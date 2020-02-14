#pragma once

#include <ros_control_boilerplate/frcrobot_hw_interface.h>

namespace frcrobot_control
{

class FRCRobotPhoenixSimInterface : public FRCRobotHWInterface
{
	public:
		FRCRobotPhoenixSimInterface(ros::NodeHandle &nh, urdf::Model *urdf_model = NULL);
		virtual bool init(ros::NodeHandle& root_nh, ros::NodeHandle &robot_hw_nh) override;

		/** \brief Read the state from the robot hardware. */
		//virtual void read(const ros::Time& time, const ros::Duration& period) override;

		/** \brief Write the command to the robot hardware. */
		virtual void write(const ros::Time& time, const ros::Duration& period) override;

};

}
