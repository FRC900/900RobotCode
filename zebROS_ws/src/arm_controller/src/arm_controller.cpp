//TO DO:
//make sure the controller_nh.param is reading talons correctly
//configure talons in YAML file correctly
//figure out service response
//process state integer to actual talon input

#include "arm_controller/arm_controller.h"

namespace arm_controller
{

bool ArmController::init(hardware_interface::RobotHW *hw,
							ros::NodeHandle			&root_nh,
							ros::NodeHandle			&controller_nh)
{
	hardware_interface::TalonCommandInterface *const talon_command_iface = hw->get<hardware_interface::TalonCommandInterface>();
	
        //read parameters and names
        if (!controller_nh.getParam("position_array", arm_positions_))
        {
            ROS_ERROR_STREAM("Could not read arm_positions");
            return false;
        }
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
        XmlRpc::XmlRpcValue arm_params;
        if(!controller_nh.getParam("arm_joint", arm_params))
        {
            ROS_ERROR_STREAM("Could not read arm_joint name");
        }

        //initialize the joint
        if (!arm_joint_.initWithNode(talon_command_iface, nullptr, controller_nh, arm_params))
        {
            ROS_ERROR("Cannot initialize arm_joint");
            return false;
        }
        else
        {
            ROS_INFO("Initialized arm_joint");
        }

        arm_joint_.setForwardSoftLimitThreshold(forward_soft_limit);
        arm_joint_.setReverseSoftLimitThreshold(reverse_soft_limit);
        arm_joint_.setForwardSoftLimitEnable(true);
        arm_joint_.setReverseSoftLimitEnable(true);
    	
	arm_state_service_ = controller_nh.advertiseService("arm_state_service", &ArmController::cmdService, this);

	return true;
}

void ArmController::starting(const ros::Time &time) {
	ROS_ERROR_STREAM("ArmController was started");
}

void ArmController::update(const ros::Time &time, const ros::Duration &period) {
	// TODO : translate from a number to an arm posttion
	// An idea - create a param which is an array.  Use the value
	// read here to index into the array. That is, the command here is the
	// index of an array, and the value at that index is the position to move
	// the arm to. Be sure to do bounds checking - make sure you don't index
	// past the end of the array.  But this will make it very easy
	// to configure different positions for the arm simply by changing a config file
        int command = *(service_command_.readFromRT());
        double position; //define in correct scope
	if (command < arm_positions_.size())
            position = arm_positions_[command];
        else 
            ROS_ERROR_STREAM("the command to arm_controller needs to be 0, 1, or 2");
        
	ROS_INFO_STREAM("arm_joint command = " << position);
	arm_joint_.setCommand(position);
}

void ArmController::stopping(const ros::Time &time) {
}

bool ArmController::cmdService(arm_controller::SetArmState::Request &req, arm_controller::SetArmState::Response &res) {
	if(isRunning())
	{
		service_command_.writeFromNonRT(req.position); //write service request
	}
	else
	{
		ROS_ERROR_STREAM("Can't accept new commands. ArmController is not running.");
		return false;
	}
	return true;
}

}//namespace

//DON'T FORGET TO EXPORT THE CLASS SO CONTROLLER_MANAGER RECOGNIZES THIS AS A TYPE
PLUGINLIB_EXPORT_CLASS( arm_controller::ArmController, controller_interface::ControllerBase)
