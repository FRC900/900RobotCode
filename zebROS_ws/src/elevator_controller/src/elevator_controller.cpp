#include "elevator_controller/elevator_controller.h"

namespace elevator_controller
{
bool ElevatorController::init(hardware_interface::RobotHW *hw,
		ros::NodeHandle                 &root_nh,
		ros::NodeHandle                 &controller_nh)
{
	//create the interface used to initialize the talon joint
	hardware_interface::TalonCommandInterface *const talon_command_iface = hw->get<hardware_interface::TalonCommandInterface>();

	//hardware_interface::PositionJointInterface *const pos_joint_iface = hw->get<hardware_interface::PositionJointInterface>();

	//get config values for the elevator talon
	XmlRpc::XmlRpcValue elevator_params;
	if (!controller_nh.getParam("elevator_joint", elevator_params))
	{ ROS_ERROR("Could not find elevator_joint");
		return false;
	}

	//initialize the elevator joint
	if(!elevator_joint_.initWithNode(talon_command_iface, nullptr, controller_nh, elevator_params))
	{
		ROS_ERROR("Cannot initialize elevator joint!");
	}

	// read elevator forward/reverse limits
	double forward_soft_limit;
	if (!controller_nh.getParam("forward_soft_limit", forward_soft_limit))
	{
		ROS_ERROR_STREAM("Could not read forward_soft_limit");
		return false;
	}
	double reverse_soft_limit;
	if (!controller_nh.getParam("reverse_soft_limit", reverse_soft_limit))
	{
		ROS_ERROR_STREAM("Could not read reverse_soft_limit");
		return false;
	}

	elevator_joint_.setForwardSoftLimitThreshold(forward_soft_limit);
	elevator_joint_.setReverseSoftLimitThreshold(reverse_soft_limit);
	elevator_joint_.setForwardSoftLimitEnable(true);
	elevator_joint_.setReverseSoftLimitEnable(true);

	elevator_service_ = controller_nh.advertiseService("elevator_service", &ElevatorController::cmdService, this);

	return true;
}

void ElevatorController::starting(const ros::Time &/*time*/) {
}

void ElevatorController::update(const ros::Time &time,const  ros::Duration &duration) 
{
	elevator_joint_.setCommand(*(position_command_.readFromRT()));
}

void ElevatorController::stopping(const ros::Time &time) {
}


//Command Service Function
bool ElevatorController::cmdService(elevator_controller::ElevatorSrv::Request &req, elevator_controller::ElevatorSrv::Response &response) {
	if(isRunning())
	{
		position_command_.writeFromNonRT(req.position);
		ROS_INFO_STREAM("writing " << std::to_string(req.position) << " to elevator controller");
	}
	else
	{
		ROS_ERROR_STREAM("Can't accept new commands. ElevatorController is not running.");
		return false;
	}
	return true;
}

}//namespace

//DON'T FORGET TO EXPORT THE CLASS SO CONTROLLER_MANAGER RECOGNIZES THIS AS A TYPE
PLUGINLIB_EXPORT_CLASS(elevator_controller::ElevatorController, controller_interface::ControllerBase)
