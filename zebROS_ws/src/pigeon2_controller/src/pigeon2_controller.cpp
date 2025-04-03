#include <controller_interface/controller.h>
#include <std_srvs/SetBool.h>
#include "pigeon2_controller/pigeon2_controller_interface.h"

namespace pigeon2_controller
{
class Pigeon2Controller: public controller_interface::Controller<hardware_interface::pigeon2::Pigeon2CommandInterface>
{
private:
	std::unique_ptr<pigeon2_controller_interface::Pigeon2ControllerInterface> interface_;
	ros::ServiceServer disable_no_motion_calibration_server;

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

	disable_no_motion_calibration_server = controller_nh.advertiseService("disable_no_motion_calibration", &Pigeon2Controller::disable_no_motion_calibration_callback, this);
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

private:
bool disable_no_motion_calibration_callback(std_srvs::SetBool::Request &msg,
											std_srvs::SetBool::Response &out_msg)
{
	ROS_INFO_STREAM("pigeon2_controller: setDisableNoMotionCalibration " << static_cast<int>(msg.data));
	interface_->setDisableNoMotionCalibration(msg.data);
	out_msg.message = "Calibrate this";
	out_msg.success = true;
	return true;
}
};

} // namespace pigeon2_controller

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(pigeon2_controller::Pigeon2Controller, controller_interface::ControllerBase)
