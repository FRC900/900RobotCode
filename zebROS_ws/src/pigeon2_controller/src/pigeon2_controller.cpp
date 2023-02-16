#include <controller_interface/controller.h>
#include "pigeon2_controller/pigeon2_controller_interface.h"

namespace pigeon2_controller
{
class Pigeon2Controller: public controller_interface::Controller<hardware_interface::pigeon2::Pigeon2CommandInterface>
{
private:
	std::unique_ptr<pigeon2_controller_interface::Pigeon2ControllerInterface> interface_;

public:
Pigeon2Controller(void) =default;

bool init(hardware_interface::pigeon2::Pigeon2CommandInterface *hw,
		  ros::NodeHandle                                     &/*root_nh*/,
		  ros::NodeHandle                                     &controller_nh) override
{
	std::string joint_name;

	if (!controller_nh.getParam("joint_name", joint_name))
	{
		ROS_ERROR_STREAM("pigeon2 controller - could not read joint_name param");
		return false;
	}
	ROS_INFO("Got joint %s in Pigeon2 controller", joint_name.c_str());

	auto pigeon2_handle = hw->getHandle(joint_name);
	interface_ = std::make_unique<pigeon2_controller_interface::Pigeon2ControllerInterface>(controller_nh, joint_name, pigeon2_handle);
	return true;
}

void starting(const ros::Time &/*time*/) override
{
}

void update(const ros::Time &/*time*/, const ros::Duration & /*period*/) override
{
	interface_->update();
}

void stopping(const ros::Time & /*time*/) override
{}

};

} // namespace pigeon2_controller

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(pigeon2_controller::Pigeon2Controller, controller_interface::ControllerBase)
