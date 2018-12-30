#pragma once

#include <ros_control_boilerplate/frcrobot_hw_interface.h>

namespace frcrobot_control
{

class FRCRobotPhoenixSimInterface : public FRCRobotHWInterface
{
	public:
		FRCRobotPhoenixSimInterface(ros::NodeHandle &nh, urdf::Model *urdf_model = NULL);
		virtual void init(void) override;

		/** \brief Read the state from the robot hardware. */
		//virtual void read(ros::Duration &elapsed_time) override;

		/** \brief Write the command to the robot hardware. */
		virtual void write(ros::Duration &elapsed_time) override;

};

}
