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
	
    	//controller_nh.param("joint_names", joint_names, std::vector<std::string>());
	//joints.resize(joint_names.size());

	//init the joint with the tci, tsi (not used), the node handle, and dynamic reconfigure (t/f)
        if (!arm_joint_.initWithNode(talon_command_iface, nullptr, controller_nh))
        {
            ROS_ERROR("Cannot initialize arm_joint");
            return false;
        }
        else
        {
            ROS_INFO("Initialized arm_joint");
        }
	//set soft limits, deadband, neutral mode, PIDF slots, acceleration and cruise velocity, all the things HERE

	/*joint_1.setPIDFSlot(0);
	joint_1.setMotionAcceleration(1); //TODO
	joint_1.setMotionCruiseVelocity(1); //TODO*/
    	
	arm_state_service = controller_nh.advertiseService("arm_state_service", &ArmController::cmdService, this);

	return true;
}

void ArmController::starting(const ros::Time &time) {
	ROS_ERROR_STREAM("ArmController was started");
}

void ArmController::update(const ros::Time &time, const ros::Duration &period) {
        arm_joint_.setCommand(*(service_command_.readFromRT()));
}

void ArmController::stopping(const ros::Time &time) {
}

bool ArmController::cmdService(arm_controller::SetArmState::Request &req, arm_controller::SetArmState::Response &res) {
	if(isRunning())
	{
		service_command_.writeFromNonRT(req.value); //write service request
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
