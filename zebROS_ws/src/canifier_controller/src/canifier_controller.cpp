#include "canifier_controller/canifier_controller.h"

namespace canifier_controller
{
bool CANifierController::init(hardware_interface::canifier::CANifierCommandInterface *hw,
		                      ros::NodeHandle                                        &/*root_nh*/,
						      ros::NodeHandle                                        &controller_nh)
{
	std::string joint_name;

	if (!controller_nh.getParam("joint_name", joint_name))
	{
		ROS_ERROR_STREAM("canifier controller - could not read joint_name param");
		return false;
	}
	ROS_INFO("Got joint %s in CANifier controller", joint_name.c_str());

	auto canifier_handle = hw->getHandle(joint_name);
	interface_ = std::make_unique<canifier_controller_interface::CANifierControllerInterface>(controller_nh, joint_name, canifier_handle);
	return true;
}

void CANifierController::starting(const ros::Time &/*time*/)
{
}

void CANifierController::update(const ros::Time &/*time*/, const ros::Duration & /*period*/)
{
	interface_->update();
}

void CANifierController::stopping(const ros::Time & /*time*/)
{}

} // namespace canifier_controller

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(canifier_controller::CANifierController, controller_interface::ControllerBase)
