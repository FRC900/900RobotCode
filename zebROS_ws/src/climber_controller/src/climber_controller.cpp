#include "climber_controller/climber_controller.h"

namespace climber_controller
{

bool ClimberController::init(hardware_interface::RobotHW *hw,
                                                        ros::NodeHandle                 &root_nh,
                                                        ros::NodeHandle                 &controller_nh)
{
    hardware_interface::PositionJointInterface *const pos_joint_iface = hw->get<hardware_interface::PositionJointInterface>();

    climber_in_ = pos_joint_iface->getHandle("climber");

    climber_service_ = controller_nh.advertiseService("climber_command", &ClimberController::activateSrv, this);

    return true;
}

void ClimberController::starting(const ros::Time &/*time*/) {
    //climber_in_.setCommand(0);
}

void ClimberController::update(const ros::Time &time, const ros::Duration &period) {
    bool climber_in_cmd = *(climber_in_cmd_.readFromRT());
    double climber_in_cmd_double;
	if(climber_in_cmd == true)
	{
		climber_in_cmd_double = -1;
	}
	else if(climber_in_cmd == false)
	{
		climber_in_cmd_double = 1;
	}
	
    climber_in_.setCommand(climber_in_cmd_double); // set the in/out command to the clampy part of the climber
}

void ClimberController::stopping(const ros::Time &time) {
}

bool ClimberController::activateSrv(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &/*response*/) {
    if(isRunning())
    {
        climber_in_cmd_.writeFromNonRT(req.data);
    }
    else
    {
        ROS_ERROR_STREAM("Can't accept new commands. ClimberController is not running.");
        return false;
    }
    return true;
}

}//namespace

//DON'T FORGET TO EXPORT THE CLASS SO CONTROLLER_MANAGER RECOGNIZES THIS AS A TYPE
PLUGINLIB_EXPORT_CLASS(climber_controller::ClimberController, controller_interface::ControllerBase)
