#include "cargo_intake_controller/cargo_intake_controller.h"

namespace cargo_intake_controller
{

bool CargoIntakeController::init(hardware_interface::RobotHW *hw,
                                                        ros::NodeHandle                 &/*root_nh*/,
                                                        ros::NodeHandle                 &controller_nh)
{
    hardware_interface::TalonCommandInterface *const talon_command_iface = hw->get<hardware_interface::TalonCommandInterface>();
    hardware_interface::PositionJointInterface *const pos_joint_iface = hw->get<hardware_interface::PositionJointInterface>();

	//initialize cargo intake arm joint (pneumatic piston that controls up/down of arm)
	cargo_intake_arm_joint_ = pos_joint_iface->getHandle("cargo_intake_arm_joint");

	//initialize cargo_intake_joint (spinny thing)
	//read cargo intake name from config file
    XmlRpc::XmlRpcValue cargo_intake_params;
    if (!controller_nh.getParam("cargo_intake_joint", cargo_intake_params))
    {
        ROS_ERROR_STREAM("Can not read cargo intake name");
        return false;
    }

	//initialize cargo intake joint
    if (!cargo_intake_joint_.initWithNode(talon_command_iface, nullptr, controller_nh, cargo_intake_params))
    {
        ROS_ERROR("Cannot initialize cargo intake joint!");
        return false;
    }
    else
    {
        ROS_INFO("Initialized cargo intake joint!");
    }

    cargo_intake_service_ = controller_nh.advertiseService("cargo_intake_command", &CargoIntakeController::cmdService, this);

	return true;
}

void CargoIntakeController::starting(const ros::Time &/*time*/) {
	// set the command to the spinny part of the intake and the command to the up/down part of the intake
	cargo_intake_cmd_.writeFromNonRT(CargoIntakeCommand(0, false));
}

void CargoIntakeController::update(const ros::Time &/*time*/, const ros::Duration &/*period*/) {
	//process input for the up/down part of the intake (pneumatic piston)
	const CargoIntakeCommand cargo_intake_cmd = *(cargo_intake_cmd_.readFromRT());
	double intake_arm_cmd_double; //to store processed input
	if(cargo_intake_cmd.intake_arm_cmd_ == true) {
		intake_arm_cmd_double = 1;
	}
	else {
		intake_arm_cmd_double = 0;
	}
	//ROS_WARN_STREAM("cargo intake arm command: " << intake_arm_cmd_double);

	//read spin command
	cargo_intake_joint_.setCommand(cargo_intake_cmd.spin_cmd_); // set the command to the spinny part of the intake
	cargo_intake_arm_joint_.setCommand(intake_arm_cmd_double); // set the in/out command to the up/down part of the intake
}

void CargoIntakeController::stopping(const ros::Time &/*time*/) {
}

bool CargoIntakeController::cmdService(cargo_intake_controller::CargoIntakeSrv::Request &req, cargo_intake_controller::CargoIntakeSrv::Response &/*res*/) {
    if(isRunning())
    {
		//take the service request for a certain amount of power (-1 to 1) and write it to the command variable
		//take the service request for in/out (true/false???) and write to a command variable
		cargo_intake_cmd_.writeFromNonRT(CargoIntakeCommand(req.power, req.intake_arm));
	}
    else
    {
        ROS_ERROR_STREAM("Can't accept new commands. CargoIntakeController is not running.");
        return false;
    }
    return true;
}

}//namespace

//DON'T FORGET TO EXPORT THE CLASS SO CONTROLLER_MANAGER RECOGNIZES THIS AS A TYPE
PLUGINLIB_EXPORT_CLASS(cargo_intake_controller::CargoIntakeController, controller_interface::ControllerBase)
