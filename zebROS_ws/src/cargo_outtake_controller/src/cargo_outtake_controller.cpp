#include "cargo_outtake_controller/cargo_outtake_controller.h"

namespace cargo_outtake_controller
{

bool CargoOuttakeController::init(hardware_interface::RobotHW *hw,
                                                        ros::NodeHandle                 &root_nh,
                                                        ros::NodeHandle                 &controller_nh)
{
    hardware_interface::PositionJointInterface *const pos_joint_iface = hw->get<hardware_interface::PositionJointInterface>();

    //initialize cargo intake arm joint (pneumatic piston that controls up/down of arm)
    cargo_outtake_arm_joint_ = pos_joint_iface->getHandle("cargo_outtake_arm_joint");
    
    cargo_outtake_service_ = controller_nh.advertiseService("cargo_outtake_command", &CargoOuttakeController::cmdService, this);
    
    return true;
}

void CargoOuttakeController::starting(const ros::Time &/*time*/) {
    cargo_outtake_arm_joint_.setCommand(-1); // set the command to the up/down part of the outtake
}

void CargoOuttakeController::update(const ros::Time &time, const ros::Duration &period) {
    //process input for the up/down part of the intake (pneumatic piston)
    bool outtake_arm_command = *(outtake_arm_command_.readFromRT());
    double outtake_arm_command_double; //to store processed input
    if(outtake_arm_command == true) {
        //ROS_WARN("cargo outtake arm command: -1");
        outtake_arm_command_double = -1;
    }
    else if (outtake_arm_command == false) {
        outtake_arm_command_double = 1;
        //ROS_WARN("cargo outtake arm command: 1");
    }

    
    cargo_outtake_arm_joint_.setCommand(outtake_arm_command_double); // set the in/out command to the up/down part of the outtake
}
	
	void CargoOuttakeController::stopping(const ros::Time &time) {

}

bool CargoOuttakeController::cmdService(cargo_outtake_controller::CargoOuttakeSrv::Request &req, cargo_outtake_controller::CargoOuttakeSrv::Response &res) {
    if(isRunning())
    {
        outtake_arm_command_.writeFromNonRT(req.outtake_arm); //take the service request for in/out (true/false???) and write to a command variable
    }
    else
    {
        ROS_ERROR_STREAM("Can't accept new commands. CargoOuttakeController is not running.");
        return false;
    }
    return true;
}

}//namespace

//DON'T FORGET TO EXPORT THE CLASS SO CONTROLLER_MANAGER RECOGNIZES THIS AS A TYPE
PLUGINLIB_EXPORT_CLASS(cargo_outtake_controller::CargoOuttakeController, controller_interface::ControllerBase)

