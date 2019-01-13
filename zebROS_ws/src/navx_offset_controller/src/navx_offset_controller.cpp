#include "navx_offset_controller/navx_offset_controller.h"

namespace navx_offset_controller
{

//define the bodies of the init, starting, update, and stopping functions which were defined in the header file

bool NavXOffsetController::init(hardware_interface::RobotHW *hw,
							ros::NodeHandle &root_nh,
							ros::NodeHandle &controller_nh)
{
	//put the handle for the navX joint in the member variable navX_joint_
	hardware_interface::PositionJointInterface *const pos_joint_iface_ = hw->get<hardware_interface::PositionJointInterface>();
	navX_joint_ = pos_joint_iface_->getHandle("navX_zero"); //navX_zero is the name of the joint as hard-coded in the hw interface

	//advertise the service
	navX_offset_service_ = controller_nh.advertiseService("navX_offset_service", &NavXOffsetController::cmdService, this);

	return true;
}

void NavXOffsetController::starting(const ros::Time &time)
{
	ROS_ERROR_STREAM("NavXOffsetController Starting");

	//set a default value?
}

void NavXOffsetController::update(const ros::Time &time, const ros::Duration &duration)
{
	//write the command to the navx joint
	navX_joint_.setCommand( *(service_command_.readFromRT()) );
}

void NavXOffsetController::stopping(const ros::Time &time)
{
	//don't need to do anything
}


//define the callback function that gets called whenever a request is sent to this service server
bool NavXOffsetController::cmdService(navx_offset_controller::NavXSrv::Request &req, navx_offset_controller::NavXSrv::Response &res)
{
	//write the request to the realtime buffer if the controller is running
	if(isRunning())
	{
		service_command_.writeFromNonRT(req.value);
	}
	else
	{
		ROS_ERROR_STREAM("Can't accept new commands. NavXOffsetController is not running");
		return false;
	}
	return true;
}

} //namespace

//DON'T FORGET TO EXPORT THE CLASS SO CONTROLLER_MANAGER RECOGNIZES THIS AS A TYPE
PLUGINLIB_EXPORT_CLASS( navx_offset_controller::NavXOffsetController, controller_interface::ControllerBase)
