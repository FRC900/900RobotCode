#include "cargo_outtake_controller/cargo_outtake_controller.h"

namespace cargo_outtake_controller
{

	bool CargoOuttakeController::init(hardware_interface::PositionJointInterface *hw,
			ros::NodeHandle                 &root_nh,
			ros::NodeHandle                 &controller_nh)
	{
		//initialize cargo outtake kicker joint (pneumatic piston that controls kicker)
		cargo_outtake_kicker_joint_ = hw->getHandle("cargo_outtake_kicker_joint");
		cargo_outtake_clamp_joint_ = hw->getHandle("cargo_outtake_clamp_joint");

		cargo_outtake_service_ = controller_nh.advertiseService("cargo_outtake_command", &CargoOuttakeController::cmdService, this);

		return true;
	}

	void CargoOuttakeController::starting(const ros::Time &/*time*/) {
		cargo_outtake_kicker_joint_.setCommand(0); // set the command to the kicker of the outtake
		cargo_outtake_clamp_joint_.setCommand(0); // set the command to the up/down part of the outtake
	}

	void CargoOuttakeController::update(const ros::Time &time, const ros::Duration &period) {
		//process input for the up/down part of the intake (pneumatic piston)
		bool kicker_command = *(kicker_command_.readFromRT());
		double kicker_command_double; //to store processed input
		if(kicker_command == true) {
			//ROS_WARN("cargo outtake kicker command: -1");
			kicker_command_double = 1;
		}
		else if (kicker_command == false) {
			kicker_command_double = 0;
			//ROS_WARN("cargo outtake kicker command: 1");
		}

		bool clamp_command = *(clamp_command_.readFromRT());
		double clamp_command_double; //to store processed input
		if(clamp_command == true) {
			//ROS_WARN("cargo outtake kicker command: -1");
			clamp_command_double = 1;
		}
		else if (clamp_command == false) {
			clamp_command_double = 0;
			//ROS_WARN("cargo outtake kicker command: 1");
		}

		cargo_outtake_kicker_joint_.setCommand(kicker_command_double); // set the in/out command to the up/down part of the outtake
		cargo_outtake_clamp_joint_.setCommand(clamp_command_double); // set the in/out command to the up/down part of the outtake
	}

	void CargoOuttakeController::stopping(const ros::Time &time) {

	}

	bool CargoOuttakeController::cmdService(cargo_outtake_controller::CargoOuttakeSrv::Request &req, cargo_outtake_controller::CargoOuttakeSrv::Response &res) {
		if(isRunning())
		{
			//kick = true, retract = false
			kicker_command_.writeFromNonRT(req.kicker_in); //take the service request for in/out (true/false???) and write to a command variable
			//clamped down = false, let go = true
			clamp_command_.writeFromNonRT(req.clamp_in); //take the service request for in/out (true/false???) and write to a command variable
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

